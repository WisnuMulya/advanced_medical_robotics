import rclpy  # Import ROS client library for Python
from rclpy.node import Node  # Import Node class from ROS Python library
from std_msgs.msg import Float32MultiArray  # Import message type for ROS
from dynamixel_sdk import *  # Import Dynamixel SDK for servo control
import numpy as np  # Import numpy for numerical operations
from rclpy.logging import LoggingSeverity  # Import LoggingSeverity for setting log levels

# Class definition for the hardware interface node
class HardwareInterfaceNode(Node):   
    # Constructor of the class
    def __init__(self):
        super().__init__('hardware_interface_ros')  # Initialize the ROS node

        self.get_logger().info('node is alive')  # Log message indicating the node is running
        self.get_logger().set_level(LoggingSeverity.ERROR)  # Increasing logging severity to avoid continuous printing to terminal

        # Dynamixel motor control related constants
        self.ADDR_TORQUE_ENABLE = 64  # Address for torque enable
        self.ADDR_OPERATING_MODE = 11  # Address for operating mode

        # Addresses for goal and current position, velocity, and current
        self.ADDR_GOAL_CURRENT = 102
        self.ADDR_GOAL_VELOCITY = 104
        self.ADDR_GOAL_POSTION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PRESENT_VELOCITY = 128
        self.ADDR_PRESENT_CURRENT = 126

        # Operating modes for the Dynamixel motors
        self.MODE_CUR = 0  # Current control mode
        self.MODE_VEL = 1  # Velocity control mode
        self.MODE_POS = 5  # Position control mode


        # Scaling factors for converting between motor and ROS values
        self.POS_SCALING = 0.087891  # Scaling for position
        self.VEL_SCALING = 0.087891*0.22888*6  # Scaling for velocity
        self.CUR_SCALING = 1  # Scaling for current


        # Limits for position, velocity, and current to prevent damage
        self.LIMIT_POS = 100
        self.LIMIT_VEL = 10
        self.LIMIT_CURRENT = 500

        self.limit_pos_tol = 1  # Tolerance for position limit checking

        self.add_on_set_parameters_callback
        self.BAUDRATE = 1000000

        self.DEVICENAME = '/dev/ttyACM0'

        self.ACTIVATE_MOTORS = True  # Flag to activate or deactive motors

        self.DXL_IDs = [1]

        self.get_logger().info("%s" % [self.DXL_IDs])

        self.joint_pos_all = np.zeros(len(self.DXL_IDs))
        self.joint_vel_all = np.zeros(len(self.DXL_IDs))
        self.joint_cur_all = np.zeros(len(self.DXL_IDs))

        self.TORQUE_ENABLE = 1     # Value for enabling the torque
        self.TORQUE_DISABLE = 0     # Value for disabling the torque

        # Initialize port and packet handlers for Dynamixel SDK
        self.portHandler = PortHandler(self.DEVICENAME)  # Port handler for communication
        self.packetHandler = PacketHandler(2.0)  # Packet handler for sending/receiving data

        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        if self.portHandler.openPort():
            self.get_logger().info('Succeeded to open the port')
        else:
            self.get_logger().info('Failed to open the port')
            self.get_logger().info('Press any key to terminate...')
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            self.get_logger().info('Succeeded to change the baudrate')
        else:
            self.get_logger().info('Failed to change the baudrate')
            self.get_logger().info('Press any key to terminate...')
            quit()

        self.operating_mode = -1  # Variable to store current operating mode
        self.set_operating_mode(self.MODE_POS)  # Set initial operating mode to position mode

        # ROS topic subscriptions for receiving desired joint states
        self.sub_angle = self.create_subscription(
            Float32MultiArray,
            '/joint_pos',
            self.desired_pos_callback,
            10
        )
        self.sub_angle # prevent unused variable warning

        self.sub_vel = self.create_subscription(
            Float32MultiArray,
            '/joint_vel',
            self.desired_vel_callback,
            10
        )
        self.sub_vel # prevent unused variable warning

        self.sub_cur = self.create_subscription(
            Float32MultiArray,
            '/joint_cur',
            self.desired_cur_callback,
            10
        )
        self.sub_cur # prevent unused variable warning

        self.sub_pos_rel = self.create_subscription(
            Float32MultiArray,
            'joint_pos_rel',
            self.joint_pos_rel_callback,
            10
        )
        self.sub_pos_rel # prevent unused variable warning

        # Publish end-effector position
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/joint_state',
            10
        )

        self.timer_period = 1.0/40.0  # Timer period for periodic callbacks
        self.timer = self.create_timer(self.timer_period, self.joint_state_callback)  

        # Initialize positions of all motors
        self.pos_0 = [30.0, -60.0, 60.0]
        for i, id in enumerate(self.DXL_IDs):
            self.set_pos(id, self.pos_0[i])



    # Set the position of a motor
    def set_pos(self, id, pos):
        self.set_operating_mode(self.MODE_POS)  # Ensure mode is set to position control

        # Limit position to prevent damage
        if pos > self.LIMIT_POS:
            pos = self.LIMIT_POS
        elif pos < -self.LIMIT_POS:
            pos = -self.LIMIT_POS

        # Convert and send position value
        pos = (pos+180.0)/self.POS_SCALING
        self.packetHandler.write4ByteTxRx(self.portHandler, id, self.ADDR_GOAL_POSTION, int(pos))

        return

    # Set the velocity of a motor
    def set_vel(self, id, vel):
        self.set_operating_mode(self.MODE_VEL)  # Ensure mode is set to velocity control

        # Limit velocity to prevent damage
        if vel > self.LIMIT_VEL:
            vel = self.LIMIT_VEL
        elif vel < -self.LIMIT_VEL:
            vel = -self.LIMIT_VEL

        # Convert and send velocity value
        vel = vel/self.VEL_SCALING
        self.packetHandler.write4ByteTxRx(self.portHandler, id, self.ADDR_GOAL_VELOCITY, int(vel))

        return
    

    # Set the current of a motor
    def set_cur(self, id, cur):
        self.set_operating_mode(self.MODE_CUR)  # Ensure mode is set to current control

        # Limit current to prevent damage
        if cur > self.LIMIT_CURRENT:
            cur = self.LIMIT_CURRENT
        elif cur < -self.LIMIT_CURRENT:
            cur = -self.LIMIT_CURRENT

        # Send current value
        self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_GOAL_CURRENT, int(cur))

        return

    # Change the operating mode of the motors
    def set_operating_mode(self, mode):
        if mode == self.operating_mode:
            return 1

        for id in self.DXL_IDs:
            # Disable torque, set mode, and re-enable torque
            self.packetHandler.write1ByteTxRx(
                self.portHandler, id, self.ADDR_TORQUE_ENABLE, 0)
            self.packetHandler.write1ByteTxRx(
                self.portHandler, id, self.ADDR_OPERATING_MODE, mode)
            
            if self.ACTIVATE_MOTORS:            
                self.packetHandler.write1ByteTxRx(
                    self.portHandler, id, self.ADDR_TORQUE_ENABLE, 1)

            # Verify mode update
            mode_actual, _, _ = self.packetHandler.read1ByteTxRx(self.portHandler, id, self.ADDR_OPERATING_MODE)
        if mode_actual == mode:
            self.operating_mode = mode_actual
            self.get_logger().info("Updated mode to %d" % self.operating_mode)
            return 1
        else:
            self.get_logger().info("Failed to update mode")
            return 0
        


    # Callback for handling desired position
    def desired_pos_callback(self, pos_msg):
        targets = pos_msg.data
        # Check if number of targets matches number of motors
        if len(targets) != len(self.DXL_IDs):
            self.get_logger().info("Number of given angles doesn't match number of motors")
            return

        # Set position for each motor
        for idx, id in enumerate(self.DXL_IDs):
            self.set_pos(id, targets[idx])

        return

    # Callback for handling desired velocity
    def desired_vel_callback(self, vel_msg):
        targets = vel_msg.data
        if len(targets) != len(self.DXL_IDs):
            self.get_logger().info("Number of given velocities doesn't match number of motors")
            return



        # Set velocity for each motor, considering joint limits
        for idx, id in enumerate(self.DXL_IDs):
            self.set_vel(id, targets[idx])
        

        # # Set velocity for each motor, considering joint limits
        # for idx, id in enumerate(self.DXL_IDs):
        #     predicted_pos = self.get_pos(id) + self.timer_period * self.get_vel(id)
        #     if (predicted_pos > self.LIMIT_POS and targets[idx] > 0.0) or \
        #        (predicted_pos < -self.LIMIT_POS and targets[idx] < 0.0):
        #         self.set_vel(id, 0.0)
        #     else:
        #         self.set_vel(id, targets[idx])
        
        return
    

    # Callback for handling desired current
    def desired_cur_callback(self, cur_msg):
        targets = cur_msg.data
        if len(targets) != len(self.DXL_IDs):
            self.get_logger().info("Number of given currents doesn't match number of motors")
            return

        # Set current for each motor
        for idx, id in enumerate(self.DXL_IDs):
            self.set_cur(id, targets[idx])

        return

    # Callback for handling relative joint position changes
    def joint_pos_rel_callback(self, pos_rel_msg):
        rel_positions = pos_rel_msg.data
        if len(rel_positions) != len(self.DXL_IDs):
            self.get_logger().info("Mismatch in the number of joints and relative position commands")
            return

        # Adjust position for each motor based on relative change
        for idx, id in enumerate(self.DXL_IDs):
            current_pos = self.get_pos(id)
            target_pos = current_pos + rel_positions[idx]
            if target_pos > self.LIMIT_POS:
                target_pos = self.LIMIT_POS
            elif target_pos < -self.LIMIT_POS:
                target_pos = -self.LIMIT_POS
            self.set_pos(id, target_pos)

        return

    # Read the current position of a motor
    def get_pos(self, id):
        pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, id, self.ADDR_PRESENT_POSITION)
        return float(pos) * self.POS_SCALING - 180.0

    # Read the current velocity of a motor
    def get_vel(self, id):
        vel, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, id, self.ADDR_PRESENT_VELOCITY)
        return float(self.s16(vel)) * self.VEL_SCALING

    # Read the current current (amperage) of a motor
    def get_cur(self, id):
        cur, _, _ = self.packetHandler.read2ByteTxRx(self.portHandler, id, self.ADDR_PRESENT_CURRENT)
        if cur > 1750:
            cur -= 2**16        
        return cur    

    def s16(self, value):
        return -(value & 0x8000) | (value & 0x7fff)


    def joint_state_callback(self):
        """
        Callback function.
        This function gets called as soon as the angle of the joints are received.
        :param: msg is of type std_msgs/Float32MultiArray 
        """
        state_msg = Float32MultiArray()

        # Update joint states
        for idx, id in enumerate(self.DXL_IDs):
            self.joint_pos_all[idx] = self.get_pos(id)
            self.joint_vel_all[idx] = self.get_vel(id)
            self.joint_cur_all[idx] = self.get_cur(id)
        
        # Check if the joint is at or beyond its limit
        if self.joint_pos_all[idx] >= self.LIMIT_POS or self.joint_pos_all[idx] <= -self.LIMIT_POS:
            self.get_logger().info(f"Joint {id} at or beyond limit: Position = {self.joint_pos_all[idx]}")


        # # Enforce hard joint limits if in velocity mode
        # if self.operating_mode == self.MODE_VEL:
        #     for idx, id in enumerate(self.DXL_IDs):
        #         predicted_pos = self.joint_pos_all[idx] + self.timer_period * self.joint_vel_all[idx]

        #         # Check if the predicted position exceeds the limits
        #         if (predicted_pos > (self.LIMIT_POS + self.limit_pos_tol)) & (self.joint_vel_all[idx] > 0.0):
        #             self.set_vel(id, 0.0)  # Stop the joint to prevent exceeding the limit

        #         elif (predicted_pos < (-self.LIMIT_POS + self.limit_pos_tol)) & (self.joint_vel_all[idx] < 0.0):
        #             self.set_vel(id, 0.0)  # Stop the joint to prevent exceeding the limit

    
        # self.get_logger().info("%s" % [(self.timer_period*self.joint_vel_all[0] + self.joint_pos_all[0]), (self.timer_period*self.joint_vel_all[1] + self.joint_pos_all[1])])

        # Populate state message
        state_msg.data.extend(self.joint_pos_all)
        state_msg.data.extend(self.joint_vel_all)
        state_msg.data.extend(self.joint_cur_all)

        self.publisher.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterfaceNode()
    #node.main_loop()
    rclpy.spin(node)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
