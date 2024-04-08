import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.logging import LoggingSeverity


class texture(Node):
    def __init__(self):
        """
        Create a texture node to act as the controller for the robot
        """
        super().__init__("texture")

        self.get_logger().info("node is alive")
        self.get_logger().set_level(LoggingSeverity.INFO)

        # Subscriptions
        self.joint_state_subscription = self.create_subscription(
            Float32MultiArray, "/joint_state", self.joint_state_callback, 10
        )

        # Publishers
        self.joint_cur_publisher = self.create_publisher(Float32MultiArray, "/joint_cur", 10)

        # link length
        self.l1 = 0.1
        self.l2 = 0.1
        self.l3 = 0.0675

        # current angle of motor 1, 2 and 3 (t1, t2 and t3)
        self.q_t1 = None
        self.q_t2 = None
        self.q_t3 = None
        self.v_t1 = None
        self.v_t2 = None
        self.v_t3 = None

        # home position and tolerance
        self.home_pos = [0.2, 0., 0]
        self.tol = 0.01

        # set constants
        self.k_spring = 0. # Spring constant
        self.c_damper = 0. # Damper constant
        self.grad1 = 0.003524380689719753 # Torque constant: the lower, the closer to the home position, but more oscillation
        self.grad2 = 0.011299511155638898
        self.grad3 = 0.006638802033504451
        self.grad = np.array([self.grad1, self.grad2, self.grad3])
        self.friction_const = 20.

        # set workspace limit 
        self.workspace_limit_x = 0.26
        self.workspace_limit_y = 0.13
        self.wall = 0.20

        self.box_centre = 0.135
        self.box_length = 0.15
        self.min_reaction_f = .5
        self.reaction_f_grad = 40.


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

    def get_f_dir(self, pos, vel):
        x = pos[0]
        y = pos[1]
        print(f'x: {x} | y: {y}')

        # calculate if position above the first line (positive gradient)
        is_above_line_1 = y > (x - self.box_centre)
        is_above_line_2 = y > (-x + self.box_centre)
        f_dir = [0., 0., 0.]
        displacement = None
        
        if is_above_line_1:
            if is_above_line_2:
                if y > self.box_length/2:
                    # top
                    f_dir = [0., -1., 0.]
                    displacement = y - self.box_length/2
                    self.k_spring = 40.
                    self.c_damper = 3.
                    self.reaction_f_grad = 40.
                else:
                    pass
            else:
                if x < self.box_centre - (self.box_length/2):
                    # right
                    f_dir = [1., 0., 0.]
                    displacement = (self.box_centre - (self.box_length/2)) - x
                    self.k_spring = 100.
                    self.c_damper = 5.
                    self.reaction_f_grad = 20.
                else:
                    pass
        else:
            if is_above_line_2:
                if x > self.box_centre + (self.box_length/2):
                    # left
                    f_dir = [-1., 0., 0.]
                    displacement = x - (self.box_centre + (self.box_length/2))
                    self.k_spring = 20.
                    self.c_damper = 1.
                    self.reaction_f_grad = 20.
                else:
                    pass
            else:
                # print(f'below line 2')
                if y < - (self.box_length/2):
                    # bottom
                    f_dir = [0., 1., 0.]
                    displacement = -(y - (-self.box_length/2))
                    self.k_spring = 40.
                    self.c_damper = 2.
                    self.reaction_f_grad = 30.
                else:
                    pass

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
        v1 = joint_state[3]
        v2 = joint_state[4]
        v3 = joint_state[5]

        self.q_t1 = t1
        self.q_t2 = t2
        self.q_t3 = t3
        self.v_t1 = v1
        self.v_t2 = v2
        self.v_t3 = v3
        q_dot = np.array([self.v_t1, self.v_t2, self.v_t3])

        # Get current task position from current joint position
        pos, orient = self.forward_kinematics(t1, t2, t3)

        # Calculate velocity in task space
        J_g = self.get_jacobian(self.q_t1, self.q_t2, self.q_t3)
        vel = J_g @ q_dot
        vel = vel[:3]

        # Determine the normal direction force
        f_dir, displacement = self.get_f_dir(pos, vel)
        print(f'displacement: {displacement}')

        if np.linalg.norm(f_dir) > 0:
            # Calculate force
            print(f'Over the wall: {f_dir}')
            # spring_term = self.k_spring * f_dir * (self.min_reaction_f + (self.reaction_f_grad * displacement))
            spring_term = self.k_spring * f_dir * (np.e**(self.reaction_f_grad * displacement) - 1)
            # spring_term = self.k_spring * f_dir * np.log((self.reaction_f_grad * displacement) + 1)
            damper_term = -self.c_damper * f_dir * np.linalg.norm(vel)
            # term_1 = self.k_spring * f_dir
            # print(f'term 1: {term_1}') 
            # term_2 = self.reaction_f_grad * displacement
            # print(f'term 2: {term_2}')

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
            for idx, val in enumerate(q_dot):
                current = i[idx]
                i[idx] = current + self.get_friction(val)

            # Publish current task position for trajectory node
            print(i)
            msg = Float32MultiArray()
            msg.data = i.tolist()
            self.joint_cur_publisher.publish(msg)
        else:
            i = [0., 0., 0.]

            # Add friction model
            for idx, val in enumerate(q_dot):
                current = i[idx]
                i[idx] = current + self.get_friction(val)

            msg = Float32MultiArray()
            msg.data = i
            self.joint_cur_publisher.publish(msg)

        # x = pos[0]
        # y = pos[1]

        # if end-effector is out of workspace
        # if pos[1:1] > self.workspace_limit_x or pos[2:1] > self.workspace_limit_y or pos[2:1] < -self.workspace_limit_y:

        # Calculate the displacement
        # self.displacement = pos - self.home_pos
        # self.displacement_eucl = np.linalg.norm(self.displacement)

        # Calculate velocity in task space
        # J_g = self.get_jacobian(self.q_t1, self.q_t2, self.q_t3)
        # vel = J_g @ q_dot
        # vel = vel[:3]

        # Calculate the force if the displacement is more than tolerance
        # print(f"Displacement: {self.displacement_eucl}")
        # print(f"Velocity: {vel}")

        # if y > x - 10:
        #     if y > -x + 10:
        #         if y > 5: # quadrant 1
        #             i = self.get_i("quadrant_1", x, y)
        #             return i
        #         else:
        #             pass
        #     else:
        #         if x < 5: # quadrant 2
        #             i = self.get_i("quadrant_2", x, y)
        #             return i
        #         else:
        #             pass
        # else:
        #     if y < -x + 10:
        #         if y < -5: # quadrant 3
        #             i = self.get_i("quadrant_1", x, y)
        #             return i
        #         else:
        #             pass
        #     else: 
        #         if x > 15: # quadrant 4
        #             i = self.get_i("quadrant_1", x, y)
        #             return i
        #         else:
        #             pass


        # if self.displacement_eucl > self.tol:
        #     f = (-self.k_spring * self.displacement) + (-self.c_damper * vel)
        #     F = f[:2]
        #     J_a = J_g[:2, :]
        #     J_t = J_a.T
        #     t = J_t @ F
        #     i = t / self.grad
        #     # i = [0, 0, 0]

        #     # Add friction model
        #     for idx, val in enumerate(q_dot):
        #         current = i[idx]
        #         i[idx] = current + self.get_friction(val) 


            


def main():
    rclpy.init()
    texture_node = texture()

    try:
        rclpy.spin(texture_node)
    except KeyboardInterrupt:
        pass

    texture_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
