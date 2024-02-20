import rclpy#, time
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Float32MultiArray

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        self.get_logger().info("node is alive")
        self.get_logger().set_level(LoggingSeverity.INFO)

        # Declares subscriber
        self.joint_state_subs = self.create_subscription(Float32MultiArray, "/joint_state", self.joint_state_callback, 10)

        # Declares publisher
        self.controller_output_pubs = self.create_publisher(Float32MultiArray, "/joint_cur", 10)

        self.kp = 5.
        self.ki = .25
        self.kd = 2.
        self.target_q_dot = 10.
        self.prev_q_dot_error = None
        # self.prev_time = None
        self.q_dot_error_total = 0.

    def get_cur_from_vel(self, vel):
        sign = 1 if vel > 0 else -1
        grad = 1.
        offset = 8.
        return (sign * offset) + (grad * vel)


    def joint_state_callback(self, msg):
        # Read velocity
        data = msg.data
        read_q_dot = data[1]

        # Calculate error for proportional term
        q_dot_error = self.target_q_dot - read_q_dot
        kp_term = self.kp * q_dot_error

        # # Calculate difference in time
        # if self.prev_time is None:
        #     dt = 1e-9
        # else:
        #     cur_time = time.time()
        #     dt = cur_time - self.prev_time

        # self.prev_time = cur_time

        # Calculate differentials term
        if self.prev_q_dot_error is None:
            delta_q_dot_error = q_dot_error
        else:
            delta_q_dot_error = q_dot_error - self.prev_q_dot_error

        self.prev_q_dot_error = q_dot_error
        kd_term = self.kd * delta_q_dot_error

        # Calculate integral term
        self.q_dot_error_total += q_dot_error

        # Clamping q_dot total error
        if self.q_dot_error_total > 100:
            self.q_dot_error_total = 100.
        elif self.q_dot_error_total < -100:
            self.q_dot_error_total = -100.

        ki_term = self.ki * self.q_dot_error_total

        # Calculate total output
        print(f'Kp: {kp_term} | Kd: {kd_term} | Ki: {ki_term}')
        output_i = kp_term + kd_term + ki_term
        
        # Create message to publish
        controller_msg = Float32MultiArray()
        controller_msg.data = [output_i]
        self.controller_output_pubs.publish(controller_msg)

def main():
    rclpy.init()
    pid_controller_node = PIDController()

    try:
        rclpy.spin(pid_controller_node)
    except KeyboardInterrupt:
        pass
    
    pid_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
