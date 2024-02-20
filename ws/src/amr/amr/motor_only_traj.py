import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Float32MultiArray

class MotorOnlyTraj(Node):
    def __init__(self):
        super().__init__('motor_only_traj')

        self.get_logger().info("node is alive")
        self.get_logger().set_level(LoggingSeverity.INFO)

        # Declares publishers
        self.vel_publisher = self.create_publisher(Float32MultiArray, "/joint_vel", 10)

        # Declares timer
        self.timer_period = 1  # Setting the timer period to .5 seconds
        self.timer = self.create_timer(
            self.timer_period, 
            self.timer_callback
        )

        self.inc = 0
        self.current_vel = 10.

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [self.current_vel]

        self.vel_publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

        # self.inc += 1
        # if self.inc % 2 == 0:
        #     self.current_vel += .5

def main():
    rclpy.init()
    motor_only_traj_node = MotorOnlyTraj()

    try:
        rclpy.spin(motor_only_traj_node)
    except KeyboardInterrupt:
        pass
    
    motor_only_traj_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
