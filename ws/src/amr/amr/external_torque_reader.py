import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Float32MultiArray

class ExternalTorque(Node):
    def __init__(self):
        super().__init__('external_torque_reader')

        self.get_logger().info("node is alive")
        self.get_logger().set_level(LoggingSeverity.INFO)

        # Declares subscriber
        self.joint_state_subs = self.create_subscription(Float32MultiArray, "/joint_state", self.joint_state_callback, 10)

        # Declares publisher
        self.ext_torque_pubs = self.create_publisher(Float32MultiArray, "/ext_torque", 10)

    def get_cur_from_vel(self, vel):
        sign = 1 if vel > 0 else -1
        grad = 1.
        offset = 8.
        return (sign * offset) + (grad * vel)


    def joint_state_callback(self, msg):
        data = msg.data
        vel = data[1]
        model_cur = self.get_cur_from_vel(vel) 
        read_cur = data[2]
        ext_cur = read_cur - model_cur
        # Create message to publish
        ext_msg = Float32MultiArray()
        ext_msg.data = [ext_cur, model_cur]
        self.ext_torque_pubs.publish(ext_msg)

def main():
    rclpy.init()
    external_torque_node = ExternalTorque()

    try:
        rclpy.spin(external_torque_node)
    except KeyboardInterrupt:
        pass
    
    external_torque_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
