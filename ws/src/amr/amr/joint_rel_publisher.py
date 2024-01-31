# Importing necessary libraries from ROS2 Python client library
import rclpy, random
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
from std_msgs.msg import Float32MultiArray  # Import message type for ROS

# Defining the SimplePublisher class which inherits from Node
class SimplePublisher(Node):

    def __init__(self):
        super().__init__('joint_rel_publisher')  # Initialize the node with the name 'simple_publisher'
        # Create a publisher object with String message type on the topic 'advanced_topic'
        # The second argument '10' is the queue size
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joint_pos_rel', 10)
        timer_period = 0.1  # Setting the timer period to .5 seconds
        # Create a timer that calls the timer_callback method every .5 second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pos_rel = [0., 0., 2.]

    def timer_callback(self):
        msg = Float32MultiArray()  # Creating a String message object
        # self.pos_rel = [n + 0.1 for n in self.pos_rel]
        msg.data = self.pos_rel  # Setting the message data
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
