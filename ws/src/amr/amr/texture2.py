import rclpy
import numpy as np
from datetime import datetime
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.logging import LoggingSeverity


class texture2(Node):
    def __init__(self):
        """
        Create a texture node to act as the controller for the robot
        """
        super().__init__("texture2")

        self.get_logger().info("node is alive")
        self.get_logger().set_level(LoggingSeverity.INFO)

        # Subscriptions
        self.joint_state_subscription = self.create_subscription(
            Float32MultiArray, "/joint_state", self.joint_state_callback, 10
        )

        # Publishers
        self.joint_cur_publisher = self.create_publisher(Float32MultiArray, "/joint_cur", 10)
        self.torque_publisher = self.create_publisher(Float32MultiArray, "/joint_torque", 10)

        # link length
        self.l1 = 0.1
        self.l2 = 0.1
        self.l3 = 0.0675

        # current angle of motor 1, 2 and 3 (t1, t2 and t3)
        self.q1 = None
        self.q2 = None
        self.q3 = None
        self.q1_dot = None
        self.q2_dot = None
        self.q3_dot = None

        # set constants
        self.k_spring = 0. # Spring constant
        self.c_damper = 0. # Damper constant
        self.grad1 = 0.003524380689719753 # Torque constant: the lower, the closer to the home position, but more oscillation
        self.grad2 = 0.011299511155638898
        self.grad3 = 0.006638802033504451
        self.grad = np.array([self.grad1, self.grad2, self.grad3])
        self.friction_const = 20.

        # set workspace limit 
        self.box_centre = 0.135
        self.box_length = 0.15
        self.min_reaction_f = .5
        self.reaction_f_grad = 40.
        
        # inertia model
        self.motor_mass = 0.018
        self.arm_mass = 0.02783
        self.m1 = 3*self.arm_mass + 2*self.motor_mass
        self.m2 = 2*self.arm_mass + self.motor_mass
        self.m3 = self.arm_mass

        # set important variables
        self.prev_time = datetime.now()
        self.prev_q_dot = None

    def forward_kinematics(self, theta1, theta2, theta3):
        """
        Calculate the forward kinematics of the robot
        """
        # Homogeneous transformation matrices
        A1 = np.array(
            [
                [np.cos(theta1), -np.sin(theta1), 0, self.l1 * np.cos(theta1)],
                [np.sin(theta1), np.cos(theta1), 0, self.l1 * np.sin(theta1)],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        A2 = np.array(
            [
                [np.cos(theta2), -np.sin(theta2), 0, self.l2 * np.cos(theta2)],
                [np.sin(theta2), np.cos(theta2), 0, self.l2 * np.sin(theta2)],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        A3 = np.array(
            [
                [np.cos(theta3), -np.sin(theta3), 0, self.l3 * np.cos(theta3)],
                [np.sin(theta3), np.cos(theta3), 0, self.l3 * np.sin(theta3)],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        # Forward kinematics
        T_0_3 = A1 @ A2 @ A3

        # Extract position and orientation
        position = T_0_3[:3, 3]
        orientation = T_0_3[:3, :3]

        return position, orientation

    def get_jacobian(self, t1, t2, t3):
        J = np.array(
            [
                [
                    -self.l1 * np.sin(t1)
                    - self.l2 * np.sin(t1 + t2)
                    - self.l3 * np.sin(t1 + t2 + t3),
                    -self.l2 * np.sin(t1 + t2) - self.l3 * np.sin(t1 + t2 + t3),
                    -self.l3 * np.sin(t1 + t2 + t3),
                ],
                [
                    self.l1
                    * np.cos(
                        t1 + self.l2 * np.cos(t1 + t2) + self.l3 * np.cos(t1 + t2 + t3)
                    ),
                    self.l2 * np.cos(t1 + t2) + self.l3 * np.cos(t1 + t2 + t3),
                    self.l3 * np.cos(t1 + t2 + t3),
                ],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [1, 1, 1],
            ]
        )

        return J

    def get_friction(self, vel):
        # friction i = ((2*friction_constant) / (1 - e^(-v))) - friction_constant
        res1 = vel * -1.
        res2 = np.e**res1
        res3 = (self.friction_const*2) / (1 + res2)

        return res3 - self.friction_const

    
    def get_dynamics(self):
        # reassign class variables to function variables
        t1 = self.q1
        t2 = self.q2
        t3 = self.q3
        t1_dot = self.q1_dot
        t2_dot = self.q2_dot
        t3_dot = self.q3_dot
        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        m1 = self.m1
        m2 = self.m2
        m3 = self.m3

        # calculate M matrix (moment of inertia)
        M = np.array([
            [
                (l1**2*m1 + l1**2*m2 + 4*l1**2*m3 + 2*l2**2*m2 + 2*l2**2*m3 + 3*l3**2*m3 + 8*l1*l3*m3*np.cos(t2 + t3) + 3*l1*l2*m2*np.cos(t2) + 6*l1*l2*m3*np.cos(t2) + 5*l2*l3*m3*np.cos(t3)),
                (2*l2**2*m2 + 2*l2**2*m3 + 3*l3**2*m3 + 2*l1*l3*m3*np.cos(t2 + t3) + l1*l2*m2*np.cos(t2) + 2*l1*l2*m3*np.cos(t2) + 5*l2*l3*m3*np.cos(t3)),
                (3*l3**2*m3 + 2*l1*l3*m3*np.cos(t2 + t3) + 2*l2*l3*m3*np.cos(t3))
            ],
            [
                (l1**2*m2 + 4*l1**2*m3 + 2*l2**2*m2 + 2*l2**2*m3 + 3*l3**2*m3 + 8*l1*l3*m3*np.cos(t2 + t3) + 3*l1*l2*m2*np.cos(t2) + 6*l1*l2*m3*np.cos(t2) + 5*l2*l3*m3*np.cos(t3)),
                (2*l2**2*m2 + 2*l2**2*m3 + 3*l3**2*m3 + 2*l1*l3*m3*np.cos(t2 + t3) + l1*l2*m2*np.cos(t2) + 2*l1*l2*m3*np.cos(t2) + 5*l2*l3*m3*np.cos(t3)),
                (3*l3**2*m3 + 2*l1*l3*m3*np.cos(t2 + t3) + 2*l2*l3*m3*np.cos(t3))
            ],
            [
                (m3*4*l1**2 + m3*2*l2**2 + m3*3*l3**2 + m3*8*l1*l3*np.cos(t2 + t3) + m3*6*l1*l2*np.cos(t2) + m3*5*l2*l3*np.cos(t3)),
                (m3*2*l2**2 + m3*3*l3**2 + m3*2*l1*l3*np.cos(t2 + t3) + m3*2*l1*l2*np.cos(t2) + m3*5*l2*l3*np.cos(t3)),
                (m3*3*l3**2 + m3*2*l1*l3*np.cos(t2 + t3) + m3*2*l2*l3*np.cos(t3))
            ]
        ])

        # calculate c matrix (centrifugal and corealis)
        c = np.array([
            (
                - 4*l1*l3*m3*t1_dot**2*np.sin(t2 + t3) - l1*l2*m2*t1_dot**2*np.sin(t2) - 2*l1*l2*m3*t1_dot**2*np.sin(t2) - 
                l2*l3*m3*t1_dot**2*np.sin(t3) - l2*l3*m3*t2_dot**2*np.sin(t3) - 4*l1*l3*m3*t1_dot*t2_dot*np.sin(t2 + t3) - 
                4*l1*l3*m3*t1_dot*t3_dot*np.sin(t2 + t3) - l1*l2*m2*t1_dot*t2_dot*np.sin(t2) - 2*l1*l2*m3*t1_dot*t2_dot*np.sin(t2) - 
                2*l2*l3*m3*t1_dot*t2_dot*np.sin(t3) - l2*l3*m3*t1_dot*t3_dot*np.sin(t3) - l2*l3*m3*t2_dot*t3_dot*np.sin(t3)
            ),
            (
                -m3*(4*l1*l3*t1_dot**2*np.sin(t2 + t3) + 2*l1*l2*t1_dot**2*np.sin(t2) + l2*l3*t1_dot**2*np.sin(t3) + 
                l2*l3*t2_dot**2*np.sin(t3) + 2*l1*l2*t1_dot*t2_dot*np.sin(t2) + 2*l2*l3*t1_dot*t2_dot*np.sin(t3) + 
                l2*l3*t1_dot*t3_dot*np.sin(t3) + l2*l3*t2_dot*t3_dot*np.sin(t3) + 4*l1*l3*t1_dot*t2_dot*np.sin(t2 + t3) + 
                4*l1*l3*t1_dot*t3_dot*np.sin(t2 + t3))
            ),
            0.
        ])

        return M, c


    def get_f_dir_line_only(self, pos, vel):
        x = pos[0]
        y = pos[1]
        f_dir = [0., 0., 0.]
        displacement = None
        if x > self.box_centre + (self.box_length/2):
            f_dir = [-1., 0., 0.]
            print("over the line")
            displacement = x - (self.box_centre + (self.box_length/2))
            if (y//.005) % 2 == 0 :
                self.k_spring = 70.
                self.c_damper = 2.
                self.reaction_f_grad = 30.
            else:
                self.k_spring = 10.
                self.c_damper = 2.
                self.reaction_f_grad = 30.
        print(f'x: {x} | y: {y}')
        return np.array(f_dir), displacement

    def get_f_dir_line_no_texture(self, pos, vel):
        x = pos[0]
        y = pos[1]
        f_dir = [0., 0., 0.]
        displacement = None
        if x > self.box_centre + (self.box_length/2):
            f_dir = [-1., 0., 0.]
            displacement = x - (self.box_centre + (self.box_length/2))
            self.k_spring = 70.
            self.c_damper = 2.
            self.reaction_f_grad = 30.
        print(f'x: {x} | y: {y}')
        return np.array(f_dir), displacement

    def joint_state_callback(self, msg):
        """
        Calculate the current from current position
        """
        # Start calculate the force needed if a displacement is detected
        joint_state = msg.data[:6]
        t1 = np.deg2rad(joint_state[0])
        t2 = np.deg2rad(joint_state[1])
        t3 = np.deg2rad(joint_state[2])
        v1 = np.deg2rad(joint_state[3])
        v2 = np.deg2rad(joint_state[4])
        v3 = np.deg2rad(joint_state[5])
        v1_deg = joint_state[3]
        v2_deg = joint_state[4]
        v3_deg = joint_state[5]

        self.q1 = t1
        self.q2 = t2
        self.q3 = t3
        self.q1_dot = v1
        self.q2_dot = v2
        self.q3_dot = v3
        q_dot = np.array([self.q1_dot, self.q2_dot, self.q3_dot])
        q_dot_deg = np.array([v1_deg, v2_deg, v3_deg])

        M, c = self.get_dynamics()

        # Update angular velocity and time
        if self.prev_q_dot is None:
            self.prev_q_dot = q_dot
            d_q_dot = np.array([0., 0., 0.])
        else:
            d_q_dot = q_dot - self.prev_q_dot
            self.prev_q_dot = q_dot

        time_now = datetime.now()
        dt = (time_now - self.prev_time).total_seconds()
        self.prev_time = time_now
        q_ddot = d_q_dot / dt # q dot dot (angular acceleration)

        # Get current task position from current joint position
        pos, orient = self.forward_kinematics(t1, t2, t3)

        # Calculate velocity in task space
        J_g = self.get_jacobian(self.q1, self.q2, self.q3)
        vel = J_g @ q_dot
        vel = vel[:3]

        # Determine the normal direction force
        f_dir, displacement = self.get_f_dir_line_no_texture(pos, vel)
        print(f'displacement: {displacement}')

        if np.linalg.norm(f_dir) > 0:
            # Calculate force
            spring_term = self.k_spring * f_dir * (np.e**(self.reaction_f_grad * displacement) - 1)
            damper_term = -self.c_damper * f_dir * np.linalg.norm(vel)
            f = spring_term + damper_term
            # print(f'force reaction: {f}')
            F = f[:2]
            J_a = J_g[:2, :]
            J_t = J_a.T
            
            # Calculate torque
            t = J_t @ F

            # Calculate current
            i = t / self.grad

            # Add friction model
            friction_t = list()
            for idx, val in enumerate(q_dot_deg):
                current = i[idx]
                i_friction = self.get_friction(val)
                t_friction = i_friction * self.grad[idx]
                friction_t.append(t_friction)
                i[idx] = current + i_friction

            # Add Inertia model
            inertia_t = M @ q_ddot
            # print(f"Inertia t: {inertia_t}")
            inertia_i = inertia_t / self.grad
            # print(f"Inertia I: {inertia_i}")
            for idx in range(len(i)):
                current = i[idx]
                i[idx] = current + inertia_i[idx]

            # Add centrifugal & corealis model
            c_i = c / self.grad
            # print(f'centrifugal & corealis: {c_i}')
            for idx in range(len(c)):
                current = i[idx]
                i[idx] = current + c[idx]

            total_t = t + friction_t + inertia_t + c
            torque_msg = Float32MultiArray()
            torque_msg.data = total_t.tolist()
            print(f"Torque msg: {torque_msg.data}")
            self.torque_publisher.publish(torque_msg)

            # Publish current task position for trajectory node
            print(i)
            msg = Float32MultiArray()
            msg.data = i.tolist()
            self.joint_cur_publisher.publish(msg)
        else:
            i = [0., 0., 0.]

            # Add friction model
            friction_t = list()
            for idx, val in enumerate(q_dot_deg):
                current = i[idx]
                i_friction = self.get_friction(val)
                t_friction = i_friction * self.grad[idx]
                friction_t.append(t_friction)
                i[idx] = current + self.get_friction(val)

            # Add Inertia model
            inertia_t = M @ q_ddot
            # print(f"Inertia t: {inertia_t}")
            inertia_i = inertia_t / self.grad
            # print(f"Inertia I: {inertia_i}")
            for idx in range(len(i)):
                current = i[idx]
                i[idx] = current + inertia_i[idx]

            # Add centrifugal & corealis model
            c_i = c / self.grad
            # print(f'centrifugal & corealis: {c_i}')
            for idx in range(len(c)):
                current = i[idx]
                i[idx] = current + c[idx]

            total_t = friction_t + inertia_t + c
            torque_msg = Float32MultiArray()
            torque_msg.data = total_t.tolist()
            print(f"Torque msg: {torque_msg.data}")
            self.torque_publisher.publish(torque_msg)

            msg = Float32MultiArray()
            msg.data = i
            self.joint_cur_publisher.publish(msg)

def main():
    rclpy.init()
    texture2_node = texture2()

    try:
        rclpy.spin(texture2_node)
    except KeyboardInterrupt:
        pass

    texture2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
