import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.logging import LoggingSeverity


class PIDMove(Node):
    def __init__(self):
        """
        Create a Kinematics node to act as the controller for the robot
        """
        super().__init__("kinematics")

        self.get_logger().info("node is alive")
        self.get_logger().set_level(LoggingSeverity.INFO)

        # Subscriptions
        self.joint_state_subscription = self.create_subscription(
            Float32MultiArray, "/joint_state", self.joint_state_callback, 10
        )
        self.joint_state_subscription

        # self.via_point_subscription = self.create_subscription(
        #     Float32MultiArray, "/via_point", self.via_point_callback, 10
        # )
        # self.via_point_subscription

        # Publishers
        self.joint_vel_rel_publisher = self.create_publisher(
            Float32MultiArray, "joint_vel_rel", 10
        )
        self.cur_pos_publisher = self.create_publisher(Float32MultiArray, "cur_pos", 10)

        self.l1 = 0.1
        self.l2 = 0.1
        self.l3 = 0.0675
        self.kp = 5.
        self.ki = .25
        self.kd = 2.
        self.cur_t1 = None
        self.cur_t2 = None
        self.cur_t3 = None
        self.init_pos = None
        self.tol = 0.005
        self.cur_pos = None
        self.target_pos = [0.2, 0.]

    def forward_kinematics(self, theta1, theta2, theta3):
        """
        Calculate the forward kinematics of the robot
        """
        # Homogeneous transformation matrices
        A1 = np.array(
            [
                [np.cos(theta1), -np.sin(theta1), 0, self.l1 * np.cos(theta1)],
                [np.sin(theta1), np.cos(theta1), 0, self.l1 * np.sin(theta1)],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        A2 = np.array(
            [
                [np.cos(theta2), -np.sin(theta2), 0, self.l2 * np.cos(theta2)],
                [np.sin(theta2), np.cos(theta2), 0, self.l2 * np.sin(theta2)],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        A3 = np.array(
            [
                [np.cos(theta3), -np.sin(theta3), 0, self.l3 * np.cos(theta3)],
                [np.sin(theta3), np.cos(theta3), 0, self.l3 * np.sin(theta3)],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        # Forward kinematics
        T_0_3 = A1 @ A2 @ A3

        # Extract position and orientation
        position = T_0_3[:3, 3]
        orientation = T_0_3[:3, :3]

        return position, orientation

    def get_jacobian(self, t1, t2, t3):
        J = np.array(
            [
                [
                    -self.l1 * np.sin(t1)
                    - self.l2 * np.sin(t1 + t2)
                    - self.l3 * np.sin(t1 + t2 + t3),
                    -self.l2 * np.sin(t1 + t2) - self.l3 * np.sin(t1 + t2 + t3),
                    -self.l3 * np.sin(t1 + t2 + t3),
                ],
                [
                    self.l1
                    * np.cos(
                        t1 + self.l2 * np.cos(t1 + t2) + self.l3 * np.cos(t1 + t2 + t3)
                    ),
                    self.l2 * np.cos(t1 + t2) + self.l3 * np.cos(t1 + t2 + t3),
                    self.l3 * np.cos(t1 + t2 + t3),
                ],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [1, 1, 1],
            ]
        )

        return J

    def get_pinv_jacobian(self, J):
        return np.linalg.pinv(J)

    def get_p_inv_damped(self, J, damping=.1): # the higher damping factor, the harder motor 3 to rotate/back to home pos
        return J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(2))

    def joint_state_callback(self, msg):
        """
        Save current joint state and move to target position if available
        """
        joint_pos = msg.data[:3]
        t1 = np.deg2rad(joint_pos[0])
        t2 = np.deg2rad(joint_pos[1])
        t3 = np.deg2rad(joint_pos[2])

        self.cur_t1 = t1
        self.cur_t2 = t2
        self.cur_t3 = t3

        pos, orient = self.forward_kinematics(t1, t2, t3)
        self.cur_pos = pos

        pos_error = self.target_pos - pos[:2]
        norm = np.linalg.norm(pos_error)

        # If not, move to it
        if norm > self.tol:
            # print(f"Moving to : {target}")
            # Get the Jacobian
            J_g = self.get_jacobian(self.cur_t1, self.cur_t2, self.cur_t3)
            J_a = J_g[:2, :]
            J_inv = self.get_p_inv_damped(J_a)

            # Get the change in joint angles
            dq = (J_inv @ pos_error)
            dq = np.rad2deg(dq).tolist()

            # Publish the change in joint angles
            msg = Float32MultiArray()
            msg.data = dq
            self.joint_vel_rel_publisher.publish(msg)


        # Publish current position for trajectory node
        msg = Float32MultiArray()
        msg.data = pos.tolist()
        self.cur_pos_publisher.publish(msg)


def main():
    rclpy.init()
    pid_move_node = PIDMove()

    try:
        rclpy.spin(pid_move_node)
    except KeyboardInterrupt:
        pass

    pid_move_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
