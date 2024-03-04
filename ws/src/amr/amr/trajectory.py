import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.logging import LoggingSeverity


class Trajectory(Node):
    def __init__(self):
        super().__init__("trajectory")

        self.get_logger().info("node is alive")
        self.get_logger().set_level(LoggingSeverity.INFO)

        # Declare subscriptions
        self.cur_x_subscription = self.create_subscription(
            Float32MultiArray, "/x", self.cur_x_callback, 10
        )
        self.x_des_subscription = self.create_subscription(
            Float32MultiArray, "/x_des", self.x_des_callback, 10
        )

        # Declares publishers
        self.x_dot_des_publisher = self.create_publisher(
            Float32MultiArray, "/x_dot_des", 10
        )

        # Declare other object vars
        self.cur_x = [0., 0.]
        self.x_tolerance = 0.005
        self.via_points_distance = 0.01
        self.via_points = list()

    # Read current robot position
    def cur_x_callback(self, msg):
        self.cur_x = msg.data

        # Check if there are via points left to move
        if self.via_points:
            cur_des = [self.via_points[0]]
            dx = np.array(cur_des) - np.array(self.cur_x)
            dx_norm = np.linalg.norm(dx)

            msg = Float32MultiArray()
            if dx_norm > self.x_tolerance:
                msg.data = cur_des.tolist()
                self.x_dot_des_publisher.publish(msg)
            else:
                msg.data = 

    # Generate via points when given desired position
    def x_des_callback(self, msg):
        x_des = np.array(msg.data)
        print(f"Receiving desired position: {x_des.tolist()}")

        while self.cur_x is None:
            continue

        # Generate via points when outside tolerance
        cur_x = np.array(self.cur_x)[:2]
        dx = np.array(x_des - cur_x)
        dx_norm = np.linalg.norm(dx)

        if dx_norm > self.x_tolerance:
            via_points_n = int(dx_norm // self.via_points_distance)
            via_points = np.ones(via_points_n)
            self.via_points = via_points.tolist()
            print(f"Via Points Generated: {via_points.tolist()}")

def main():
    rclpy.init()
    trajectory_node = Trajectory()

    try:
        rclpy.spin(trajectory_node)
    except KeyboardInterrupt:
        pass

    trajectory_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
