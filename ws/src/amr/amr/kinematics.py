import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.logging import LoggingSeverity

class Kinematics(Node):
    def __init__(self):
        super().__init__('kinematics')

        self.get_logger().info('node is alive')
        self.get_logger().set_level(LoggingSeverity.ERROR)

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/joint_state',
            self.joint_state_callback,
            10
        )
        self.subscription
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joint_pos_rel', 10)

        self.l1 = .1
        self.l2 = .1
        self.l3 = .1
        self.init_pos = None
        self.tol = .005

    def forward_kinematics(self, theta1, theta2, theta3):
        # Homogeneous transformation matrices
        A1 = np.array([[np.cos(theta1), -np.sin(theta1), 0, self.l1*np.cos(theta1)],
                    [np.sin(theta1), np.cos(theta1), 0, self.l1*np.sin(theta1)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

        A2 = np.array([[np.cos(theta2), -np.sin(theta2), 0, self.l2*np.cos(theta2)],
                    [np.sin(theta2), np.cos(theta2), 0, self.l2*np.sin(theta2)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

        A3 = np.array([[np.cos(theta3), -np.sin(theta3), 0, self.l3*np.cos(theta3)],
                    [np.sin(theta3), np.cos(theta3), 0, self.l3*np.sin(theta3)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

        # Forward kinematics
        T_0_3 = np.dot(A1, np.dot(A2, A3))

        # Extract position and orientation
        position = T_0_3[:3, 3]
        orientation = T_0_3[:3, :3]

        return position, orientation

    def get_jacobian(self, t1, t2, t3):
        J = np.array([[-self.l1*np.sin(t1)-self.l2*np.sin(t1+t2)-self.l3*np.sin(t1+t2+t3), -self.l2*np.sin(t1+t2)-self.l3*np.sin(t1+t2+t3), -self.l3*np.sin(t1+t2+t3)],
                      [self.l1*np.cos(t1+self.l2*np.cos(t1+t2)+self.l3*np.cos(t1+t2+t3)), self.l2*np.cos(t1+t2)+self.l3*np.cos(t1+t2+t3), self.l3*np.cos(t1+t2+t3)],
                      [0, 0, 0],
                      [0, 0, 0],
                      [0, 0, 0],
                      [1, 1, 1]])

        return J

    def get_pinv_jacobian(self, J):
        return np.linalg.pinv(J)

    def joint_state_callback(self, msg):
        joint_pos = msg.data[:3]
        t1 = np.deg2rad(joint_pos[0])
        t2 = np.deg2rad(joint_pos[1])
        t3 = np.deg2rad(joint_pos[2])

        pos, orient = self.forward_kinematics(t1,t2,t3)

        if self.init_pos is None:
            self.init_pos = pos

        # pos error
        pos_error = self.init_pos - pos
        is_below_tol = pos_error < self.tol
        print(is_below_tol)

        if not np.all(is_below_tol):
            J_g = self.get_jacobian(t1, t2, t3)
            J_a = J_g[:2,:]
            J_inv = self.get_pinv_jacobian(J_a)

            dq = np.dot(J_inv, pos_error[:2])
            print(dq)
            dq = np.rad2deg(dq).tolist()
            print(dq)

            msg = Float32MultiArray()
            msg.data = dq
            self.publisher_.publish(msg)

def main():
    rclpy.init()
    kinematics_node = Kinematics()

    try:
        rclpy.spin(kinematics_node)
    except KeyboardInterrupt:
        pass

    kinematics_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()