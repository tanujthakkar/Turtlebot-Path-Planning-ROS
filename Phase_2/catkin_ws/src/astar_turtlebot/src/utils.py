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

def get_yaw(Quaternion):
    roll, pitch, yaw = euler_from_quaternion([Quaternion.x, Quaternion.y, Quaternion.z, Quaternion.w])
    print(roll, pitch, yaw)
    return yaw

def get_pose(state):
    position = Point()
    position.x = state[0]
    position.y = state[1]
    position.z = 0

    quat = Quaternion()
    quat = quaternion_from_euler(0, 0, state[2])

    return position, quat