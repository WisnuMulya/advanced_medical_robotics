import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.logging import LoggingSeverity

class Trajectory(Node):
    def __init__(self):
        super().__init__('trajectory')

        self.get_logger().info('node is alive')
        self.get_logger().set_level(LoggingSeverity.INFO)

        # Declare subscriptions
        self.cur_pos_subscription = self.create_subscription(
            Float32MultiArray,
            '/cur_pos',
            self.cur_pos_callback,
            10
        )
        self.cur_pos_subscription

        self.target_pos_subscription = self.create_subscription(
            Float32MultiArray,
            '/target_pos',
            self.target_pos_callback,
            10
        )
        self.target_pos_subscription

        # Declares publishers
        self.dx_publisher = self.create_publisher(Float32MultiArray, 'dx', 10)

        # Declare other object vars
        self.cur_pos = None
        self.tolerance = 0.005

    def cur_pos_callback(self, msg):
        self.cur_pos = msg.data
        
    def target_pos_callback(self, msg):
        target_pos = np.array(msg.data)
        print(f"Receiving target position: {target_pos.tolist()}")
        
        while self.cur_pos is None:
            continue
        
        # Do trajectory when outside tolerance
        dx = np.array(target_pos - np.array(self.cur_pos)[:2])
        print(dx)
        dx_norm = np.linalg.norm(dx)

        if dx_norm > self.tolerance:
            t_list = np.arange(0, 1, 0.01)
            for t in t_list:
                tdx = t * dx
                print(tdx.tolist())
                
                # Send tdx for forward feedback
                msg = Float32MultiArray()
                msg.data = tdx.tolist()
                self.dx_publisher.publish(msg)

def main():
    rclpy.init()
    trajectory_node = Trajectory()

    try:
        rclpy.spin(trajectory_node)
    except KeyboardInterrupt:
        pass

    trajectory_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()