# Importing necessary libraries from ROS2 Python client library
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Defining the SimplePublisher class which inherits from Node
class TargetPosPub(Node):

    def __init__(self):
        
        super().__init__('target_pos')
        
        # Declare publishers
        self.x_des_publisher = self.create_publisher(
            Float32MultiArray, '/x_des', 10
        )

        # Declare timer
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target_pos = [0.10, 0.10]
        self.target_set = False

    def timer_callback(self):
        if not self.target_set:
            msg = Float32MultiArray()
            msg.data = self.target_pos
            self.x_des_publisher.publish(msg)

            # Logging the published message to the console
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.target_set = True

# The main function which serves as the entry point for the program
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    target_pos_publisher = TargetPosPub()  # Create an instance of the SimplePublisher

    try:
        rclpy.spin(target_pos_publisher)  # Keep the node alive and listening for messages
    except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
        pass

    target_pos_publisher.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown the ROS2 Python client library

# This condition checks if the script is executed directly (not imported)
if __name__ == '__main__':
    main()  # Execute the main function
