# Importing necessary libraries from ROS2 Python client library
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity  # Import LoggingSeverity for setting log levels

# Defining the LoggingDemoNode class which inherits from Node
class LoggingDemoNode(Node):

    def __init__(self):
        super().__init__('logging_demo')  # Initialize the node with the name 'logging_demo'
        
        # Set the logger's verbosity level to DEBUG to see all types of log messages
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # Logging various types of messages at different severity levels
        self.get_logger().debug('This is a DEBUG message.')  # Debug-level message
        self.get_logger().info('This is an INFO message.')   # Info-level message
        self.get_logger().warn('This is a WARN message.')    # Warning-level message
        self.get_logger().error('This is an ERROR message.')  # Error-level message
        self.get_logger().fatal('This is a FATAL message.')  # Fatal-level message

# The main function which serves as the entry point for the program
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    logging_demo_node = LoggingDemoNode()  # Create an instance of the LoggingDemoNode

    # Spin briefly to allow the logger to process the messages
    rclpy.spin_once(logging_demo_node, timeout_sec=0.1)

    logging_demo_node.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown the ROS2 Python client library

# This condition checks if the script is executed directly (not imported)
if __name__ == '__main__':
    main()  # Execute the main function
