import numpy as np
l1 = .1
l2 = .1
l3 = .1
theta1 = 0
theta2 = 0
theta3 = 90

# Homogeneous transformation matrices
A1 = np.array([[np.cos(np.deg2rad(theta1)), -np.sin(np.deg2rad(theta1)), 0, l1*np.cos(np.deg2rad(theta1))],
                [np.sin(np.deg2rad(theta1)), np.cos(np.deg2rad(theta1)), 0, l1*np.sin(np.deg2rad(theta1))],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

A2 = np.array([[np.cos(np.deg2rad(theta2)), -np.sin(np.deg2rad(theta2)), 0, l2*np.cos(np.deg2rad(theta2))],
                [np.sin(np.deg2rad(theta2)), np.cos(np.deg2rad(theta2)), 0, l2*np.sin(np.deg2rad(theta2))],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

A3 = np.array([[np.cos(np.deg2rad(theta3)), -np.sin(np.deg2rad(theta3)), 0, l3*np.cos(np.deg2rad(theta3))],
                [np.sin(np.deg2rad(theta3)), np.cos(np.deg2rad(theta3)), 0, l3*np.sin(np.deg2rad(theta3))],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

# Forward kinematics
T_0_3 = np.dot(A1, np.dot(A2, A3))

# Extract position and orientation
position = T_0_3[:3, 3]
orientation = T_0_3[:3, :3]
print(position)

