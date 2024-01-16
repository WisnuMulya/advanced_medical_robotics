# Importing necessary libraries from ROS2 Python client library
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type from standard ROS2 message library

# Defining the SimplePublisher class which inherits from Node
class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')  # Initialize the node with the name 'simple_publisher'
        # Create a publisher object with String message type on the topic 'advanced_topic'
        # The second argument '10' is the queue size
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1  # Setting the timer period to 1 second
        # Create a timer that calls the timer_callback method every 1 second
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()  # Creating a String message object
        msg.data = 'Hello ROS2 from Publisher, hi!'  # Setting the message data
        self.publisher_.publish(msg)  # Publishing the message
        # Logging the published message to the console
        self.get_logger().info('Publishing: "%s"' % msg.data)

# The main function which serves as the entry point for the program
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    simple_publisher = SimplePublisher()  # Create an instance of the SimplePublisher

    try:
        rclpy.spin(simple_publisher)  # Keep the node alive and listening for messages
    except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
        pass

    simple_publisher.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown the ROS2 Python client library

# This condition checks if the script is executed directly (not imported)
if __name__ == '__main__':
    main()  # Execute the main function
