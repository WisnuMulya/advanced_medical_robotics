import rclpy, time
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Float32MultiArray

class CurrentMotor(Node):
    def __init__(self):
        super().__init__('current_motor')

        self.get_logger().info("node is alive")
        self.get_logger().set_level(LoggingSeverity.INFO)

        # Declares publishers
        self.cur_publisher = self.create_publisher(Float32MultiArray, "/joint_cur", 10)
        # self.pos_1_publisher = self.create_publisher(Float32MultiArray, "/joint_1_pos", 10)

        # Declares timer
        self.timer_period = 1  # Setting the timer period to .5 seconds
        self.timer = self.create_timer(
            self.timer_period, 
            self.timer_callback
        )

        self.cur = 0.

    def timer_callback(self):
        msg = Float32MultiArray()
        # pos_msg = Float32MultiArray()
        msg.data = [0. , 0., self.cur]
        # pos_msg.data = [0., 0., 0.]

        self.cur_publisher.publish(msg)
        # self.pos_1_publisher.publish(pos_msg)
        self.get_logger().info(f"Publishing: {msg.data}")

        self.cur += 5
        time.sleep(1.)


def main():
    rclpy.init()
    current_motor_node = CurrentMotor()

    try:
        rclpy.spin(current_motor_node)
    except KeyboardInterrupt:
        pass
    
    current_motor_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
