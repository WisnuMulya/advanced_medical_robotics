import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.logging import LoggingSeverity


class Kinematics(Node):
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

        self.via_point_subscription = self.create_subscription(
            Float32MultiArray, "/via_point", self.via_point_callback, 10
        )
        self.dx_subscription

        # Publishers
        self.joint_pos_rel_publisher = self.create_publisher(
            Float32MultiArray, "joint_pos_rel", 10
        )
        self.cur_pos_publisher = self.create_publisher(Float32MultiArray, "cur_pos", 10)

        self.l1 = 0.1
        self.l2 = 0.1
        self.l3 = 0.1
        self.cur_t1 = None
        self.cur_t2 = None
        self.cur_t3 = None
        self.init_pos = None
        self.tol = 0.005
        self.cur_pos = None
        self.target_pos = list()

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

        # If there is a target position, move to it
        if self.target_pos:
            # Check if the target position is within tolerance
            target = self.target_pos[0]
            pos_error = target - pos[:2]
            norm = np.linalg.norm(pos_error)

            # If not, move to it
            if norm > self.tol:
                # Get the Jacobian
                J_g = self.get_jacobian(self.cur_t1, self.cur_t2, self.cur_t3)
                J_a = J_g[:2, :]
                J_inv = self.get_pinv_jacobian(J_a)

                # Get the change in joint angles
                dq = J_inv @ pos_error
                dq = np.rad2deg(dq).tolist()

                # Publish the change in joint angles
                msg = Float32MultiArray()
                msg.data = dq
                self.joint_pos_rel_publisher.publish(msg)

            # If within tolerance, remove target position from list
            else:
                self.target_pos.pop(0)
                print(f"Successfully move to {target}")

        # Publish current position for trajectory node
        msg = Float32MultiArray()
        msg.data = pos.tolist()
        self.cur_pos_publisher.publish(msg)

    def via_point_callback(self, msg):
        """
        Get via point from trajectory node and store it in self.target_pos lists
        """
        via_point = np.array(msg.data)
        print(f"Target received: {via_point}")
        self.target_pos.append(via_point)


def main():
    rclpy.init()
    kinematics_node = Kinematics()

    try:
        rclpy.spin(kinematics_node)
    except KeyboardInterrupt:
        pass

    kinematics_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
