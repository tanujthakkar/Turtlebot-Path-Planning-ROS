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
import rospy
import sys
import os
import numpy as np
import argparse
import time
import math
from Queue import PriorityQueue

# Importing messages
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray, Twist
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
import jsk_recognition_msgs.msg
import tf
from tf import TransformListener


class PurePursuit:

    def __init__(self, path, Kp=0.3, ld=0.3, Kf=0.1, max_vel=0.2, max_ang_vel=0.3):
        self.path = path # Path to track
        self.Kp = Kp # Proportional Gain
        self.ld = ld # Look-Forward Distance
        self.Kf = Kf # Look-Forward Gain
        self.max_vel = max_vel # Max Linear Velocity
        self.max_ang_vel = max_ang_vel # Max Angular Velocity

        self.turtlebot_listener = TransformListener()
        self.turtlebot_listener.waitForTransform("/map", "/base_footprint", rospy.Time(0), rospy.Duration(3.0))
        self.turtlebot_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def track_path(self):

        while(True):
            try:
                (trans, quat) = self.turtlebot_listener.lookupTransform("/map", "/base_footprint", rospy.Time.now())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            print(trans)
            print(quat)