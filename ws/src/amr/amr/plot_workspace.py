import rclpy, sys
import numpy as np
import pyqtgraph as pg
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.logging import LoggingSeverity
from PyQt5.QtWidgets import QApplication
from PyQt5 import QtWidgets


class PlotWorkspace(Node):
    def __init__(self):
        super().__init__("plot_workspace")

        self.get_logger().info("node is alive")
        self.get_logger().set_level(LoggingSeverity.INFO)

        # Subscription
        self.joint_state_subscription = self.create_subscription(
            Float32MultiArray, "/joint_state", self.joint_state_callback, 10
        )

        # Save important variables
        self.link_length = [10., 10., 6.75]
        self.box_centre = 13.5
        self.box_length = 15

        # PyQtGraph setup
        self.app = QApplication(sys.argv)
        self.win = pg.GraphicsLayoutWidget(show=True, title="3R Robot Arm Visualization")
        self.plot = self.win.addPlot(title="3D Robot Arm")

        # Set the scale to be linearly constant
        self.plot.setXRange(0, 30, padding=0)
        self.plot.setYRange(-30, 30, padding=0)
        self.plot.setAspectLocked(lock=True, ratio=1)
        self.plot.invertX(True) # invert the x axis

        # draw a rectangle background
        self.rect = QtWidgets.QGraphicsRectItem(self.box_centre - (self.box_length/2), - self.box_length/2, self.box_length, self.box_length)
        self.rect.setPen(pg.mkPen("g", width=2))  # Set the color and line width of the rectangle
        self.plot.addItem(self.rect)

        # draw vertical line
        self.vLine = pg.InfiniteLine(pos=self.box_centre+self.box_length/2, angle=90, pen={'color': 'g', 'width': 2})
        self.plot.addItem(self.vLine)

        # Set the arm plot
        self.arm_lines = None



    def joint_state_callback(self, msg):
        joint_state = msg.data[:3]
        t1 = np.deg2rad(joint_state[0])
        t2 = np.deg2rad(joint_state[1])
        t3 = np.deg2rad(joint_state[2])
        
        joint_rads = [t1, t2, t3]
        cum_joints = np.cumsum(joint_rads)

        # Initialize positions
        x_positions = [0]
        y_positions = [0]

        # Calculate positions for each joints
        for i in range(3):
            x_positions.append(x_positions[-1] + np.cos(cum_joints[i]) * self.link_length[i])
            y_positions.append(y_positions[-1] + np.sin(cum_joints[i]) * self.link_length[i])

        # Update the plot
        if self.arm_lines is None:
            # First time setup of the plot
            self.arm_lines = self.plot.plot(x_positions, y_positions, pen={"color": "r", "width": 2})
        else:
            # Update the existing plot
            self.arm_lines.setData(x_positions, y_positions)

        QApplication.processEvents()

    def close_pyqt(self):
        QApplication.quit()

def main(args=None):
    rclpy.init()
    plot_workspace_node = PlotWorkspace()

    try:
        rclpy.spin(plot_workspace_node)
    except KeyboardInterrupt:
        pass

    plot_workspace_node.close_pyqt()
    plot_workspace_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
