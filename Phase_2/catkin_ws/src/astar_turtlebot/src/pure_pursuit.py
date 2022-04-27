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
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray, PointStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
import jsk_recognition_msgs.msg
import tf
from tf import TransformListener

from utils import *

class PurePursuit:

    def __init__(self, path, goal_state, Kp=0.2, ld=0.1, Kf=0.1, max_vel=0.2, max_ang_vel=0.3):
        self.path = path # Path to track
        self.goal_state = goal_state
        self.Kp = Kp # Proportional Gain
        self.ld = ld # Look-Forward Distance
        self.Kf = Kf # Look-Forward Gain
        self.max_vel = max_vel # Max Linear Velocity
        self.max_ang_vel = max_ang_vel # Max Angular Velocity

        self.turtlebot_listener = TransformListener()
        self.turtlebot_listener.waitForTransform("/map", "/base_footprint", rospy.Time(0), rospy.Duration(2.0))

        self.turtlebot_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.target_point_pub = rospy.Publisher('pure_pursuit/target_point', PointStamped, queue_size=1)

    def track_path(self):

        idx = 0
        length = len(self.path.poses)
        print("Path lenght: {}".format(length))

        while(True):
            try:
                (trans, quat) = self.turtlebot_listener.lookupTransform("/map", "/base_footprint", rospy.Time.now())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            quaternion = Quaternion()
            quaternion.x = quat[0]
            quaternion.y = quat[1]
            quaternion.z = quat[2]
            quaternion.w = quat[3]
            x, y, _ = trans
            yaw = get_yaw(quaternion)
            # print(x, y, yaw)

            lf = self.Kf * self.max_vel + self.ld

            if(abs(length - idx) < 4):
                if(np.sqrt((x - self.path.poses[length-1].pose.position.x)**2 + (y - self.path.poses[length-1].pose.position.y)**2) < 0.2):
                    print("Path tracking complete!")
                    turtlebot_vel = Twist()
                    turtlebot_vel.linear.x = 0
                    turtlebot_vel.angular.z = 0
                    self.turtlebot_vel_pub.publish(turtlebot_vel)
                    return True

            m = np.sqrt((x - self.path.poses[length-1].pose.position.x)**2 + (y - self.path.poses[length-1].pose.position.y)**2)
            # print(m)
            for i in range(idx, min(idx+3, length)):
                # print(idx, idx+5)
                # print(m, np.sqrt((x - self.path.poses[i].pose.position.x)**2 + (y - self.path.poses[i].pose.position.y)**2))
                if(np.sqrt((x - self.path.poses[i].pose.position.x)**2 + (y - self.path.poses[i].pose.position.y)**2) < m):
                    m = np.sqrt((x - self.path.poses[i].pose.position.x)**2 + (y - self.path.poses[i].pose.position.y)**2)
                    idx = i

                if(np.sqrt((x - self.path.poses[i].pose.position.x)**2 + (y - self.path.poses[i].pose.position.y)**2) > 0.1):
                    idx = i
                    break

            tx = self.path.poses[idx].pose.position.x
            ty = self.path.poses[idx].pose.position.y
            target_point = PointStamped()
            target_point.header.stamp = rospy.Time.now()
            target_point.header.frame_id = "map"
            target_point.point.x = tx
            target_point.point.y = ty
            self.target_point_pub.publish(target_point)

            alpha = math.atan2((ty - y), (tx - x)) - yaw
            delta = math.atan2(2 * self.max_vel * np.sin(alpha), lf)

            turtlebot_vel = Twist()
            turtlebot_vel.linear.x = self.max_vel
            turtlebot_vel.angular.z = delta
            print("lin: {}, ang: {}".format(turtlebot_vel.linear.x, turtlebot_vel.angular.z))

            self.turtlebot_vel_pub.publish(turtlebot_vel)