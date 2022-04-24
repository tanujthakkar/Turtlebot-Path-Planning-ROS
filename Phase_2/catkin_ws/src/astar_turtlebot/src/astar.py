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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

sys.path.append('/usr/lib/python2.7/dist-packages')
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages/')
from tf.transformations import euler_from_quaternion

from utils import *


class Node():
    '''
        Class to represent nodes in graph search

        Attributes
        state: state of the node
        cost: cost of the node
        index: index of the node
        parent_index: index of parent node
    '''

    def __init__(self, state, cost, index, parent_index):
        self.state = state
        self.cost = cost
        self.index = index
        self.parent_index = parent_index


class AStar:

    def __init__(self):
        self.start_pose_pub = rospy.Publisher('start_pose', PoseStamped,  queue_size=10)
        self.goal_pose_pub = rospy.Publisher('goal_pose', PoseStamped,  queue_size=10)

        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.callback_start_pose)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.callback_goal_pose)

    def callback_start_pose(self, start_pose):
        self.start_pose = start_pose
        yaw = get_yaw(start_pose.pose.pose.orientation)

        self.start_state = (start_pose.pose.pose.position.x,
                            start_pose.pose.pose.position.y,
                            yaw)
        
        start_pose_msg = PoseStamped()
        start_pose_msg.header.stamp = rospy.Time.now()
        start_pose_msg.header.frame_id = "map"
        start_pose_msg.pose.position = start_pose.pose.pose.position
        start_pose_msg.pose.orientation = start_pose.pose.pose.orientation
        self.start_pose_pub.publish(start_pose_msg)

    def callback_goal_pose(self, goal_pose):
        self.goal_pose = goal_pose
        yaw = get_yaw(goal_pose.pose.orientation)

        self.goal_state = (goal_pose.pose.position.x,
                           goal_pose.pose.position.y,
                           yaw)

        # goal_pose_msg = PoseStamped()
        # goal_pose_msg.header.stamp = rospy.Time.now()
        # goal_pose_msg.header.frame_id = "map"
        # goal_pose_msg.pose.position = goal_pose.pose.position
        # goal_pose_msg.pose.orientation = goal_pose.pose.orientation
        self.goal_pose_pub.publish(goal_pose)