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
        # self.joint_state_subs = self.create_subscription(Float32MultiArray, "/joint_state", self.joint_state_callback, 10)
        self.joint_vel_rel_subscription = self.create_subscription(
            Float32MultiArray, "/joint_vel_rel", self.joint_vel_rel_callback, 10
        )

        # Declares publisher
        self.controller_output_pubs = self.create_publisher(Float32MultiArray, "/joint_cur", 10)

        self.kp = 5.
        self.ki = .25
        self.kd = 2.
        self.target_q_dot = 10.
        self.prev_q_dot_error = [None, None, None]
        # self.prev_time = None
        self.q_dot_error_total = [0., 0., 0.]


    def joint_vel_rel_callback(self, msg):
        # Read velocity
        q_dot_error = msg.data

        # Calculate i for each motor
        output_i = [0., 0., 0.]
        for idx in range(3):
            # Calculate error for proportional term
            kp_term = self.kp * q_dot_error[idx]

            # Calculate differentials term
            if self.prev_q_dot_error[idx] is None:
                delta_q_dot_error = q_dot_error[idx]
            else:
                delta_q_dot_error = q_dot_error[idx] - self.prev_q_dot_error[idx]

            self.prev_q_dot_error[idx] = q_dot_error[idx]
            kd_term = self.kd * delta_q_dot_error

            # Calculate integral term
            self.q_dot_error_total[idx] += q_dot_error[idx]

            # Clamping q_dot total error
            if self.q_dot_error_total[idx] > 100:
                self.q_dot_error_total[idx] = 100.
            elif self.q_dot_error_total[idx] < -100:
                self.q_dot_error_total[idx] = -100.

            ki_term = self.ki * self.q_dot_error_total[idx]

            # Calculate total output
            output_i[idx] = kp_term + kd_term + ki_term
        
        # Create message to publish
        print(f'PID output: {output_i}')
        controller_msg = Float32MultiArray()
        controller_msg.data = output_i
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
