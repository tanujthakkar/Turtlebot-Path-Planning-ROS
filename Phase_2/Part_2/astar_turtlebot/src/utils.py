#!/usr/env/bin python3

"""
ENPM661 Spring 2022: Planning for Autonomous Robots
Project 3 - Phase 2: A* for Turtlebot

Author(s):
Tanuj Thakkar (tanuj@umd.edu)
M. Engg Robotics
University of Maryland, College Park
"""

# Importing Modules
import sys
import os
import numpy as np
import math
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def pi_2_pi(theta):

        while(theta > np.pi):
            theta -= 2.0 * np.pi

        while(theta < -np.pi):
            theta += 2.0 * np.pi

        return theta

def to_deg(theta):
    return (theta * 180 / np.pi)

def to_rad(theta):
    return (theta / 180 * np.pi)

def truncate(number, digits):
    pow10 = 10 ** digits
    return number * pow10 // 1 / pow10

def get_yaw(Quaternion):
    roll, pitch, yaw = euler_from_quaternion([Quaternion.x, Quaternion.y, Quaternion.z, Quaternion.w])
    return yaw

def get_pose(state):
    position = [state[0], state[1], 0]
    quat = Quaternion()
    quat = list(quaternion_from_euler(0, 0, state[2]))
    print(quat)

    return position, quat