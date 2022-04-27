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


class OpenLoop:

    def __init__(self, cmds):
        self.cmds = cmds

        self.turtlebot_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def track_path(self):

        rate = rospy.Rate(1)

        for cmd in self.cmds:
            turtlebot_vel = Twist()
            turtlebot_vel.linear.x = cmd[0][0]
            turtlebot_vel.angular.z = -cmd[1][0]
            print("lin: {}, ang: {}".format(turtlebot_vel.linear.x, turtlebot_vel.angular.z))

            self.turtlebot_vel_pub.publish(turtlebot_vel)
            rate.sleep()

        turtlebot_vel.linear.x = 0
        turtlebot_vel.angular.z = 0
        print("lin: {}, ang: {}".format(turtlebot_vel.linear.x, turtlebot_vel.angular.z))

        self.turtlebot_vel_pub.publish(turtlebot_vel)