import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from hx711 import HX711
import RPi.GPIO as GPIO
import time

class ContactForcePublisher(Node):
    def __init__(self):
        super().__init__('contact_force_publisher')
        self.publisher_ = self.create_publisher(Float32, 'contact_force', 10)
        self.timer = self.create_timer(1.0/5.0, self.timer_callback) # 10Hz
        self.hx = HX711(dout_pin=24, pd_sck_pin=23)
        self.k = .49035/42446.5 # in Newton
        self.ss_v = 15348.0 # steady state value
        self.p_v = -60431.2655 # steady state + plate weight value
        # self.hx.reset()
        # self.hx.tare()

    def timer_callback(self):
        raw_value = self.hx.get_raw_data(times=2)[0]
        raw_value = -1 * (raw_value - self.ss_v)
        weight = self.k * raw_value

        msg = Float32()
        msg.data = float(weight)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {weight} | Raw Value: {raw_value}')

        time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    contact_force_publisher = ContactForcePublisher()
    rclpy.spin(contact_force_publisher)
    contact_force_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
