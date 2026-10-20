"""ROS-independent planar pose operations."""
import math
import numpy as np
def se2(pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0., 0., 1.]])


def pose2(matrix):
    return np.array([matrix[0, 2], matrix[1, 2], math.atan2(matrix[1, 0], matrix[0, 0])])
