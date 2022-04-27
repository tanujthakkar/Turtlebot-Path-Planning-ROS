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
try:
    from Queue import PriorityQueue
except Exception:
    from queue import PriorityQueue

# Importing messages
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray, PointStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
import tf
from tf import TransformListener

# sys.path.append('/usr/lib/python2.7/dist-packages')
# sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages/')
from tf.transformations import euler_from_quaternion

from utils import *
from pure_pursuit import *


class Node:
    '''
        Class to represent nodes in graph search

        Attributes
        state: state of the node
        cost: cost of the node
        index: index of the node
        parent_index: index of parent node
    '''

    def __init__(self, state, path, cmds, cost, index, parent_index):
        self.state = state
        self.path = path
        self.cmds = cmds
        self.cost = cost
        self.index = index
        self.parent_index = parent_index


class AStar:

    def __init__(self, start_state, RPM1, RPM2, clearance, threshold, step_size, dt, theta_step):
        self.start_pose_pub = rospy.Publisher('start_pose', PoseStamped,  queue_size=10)
        self.goal_point_pub = rospy.Publisher('goal_point', PointStamped,  queue_size=10)
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped,  queue_size=10)
        self.visited_poses_pub = rospy.Publisher('visited_poses', PoseArray,  queue_size=10)
        self.path_pub = rospy.Publisher('path', Path, queue_size=10)

        self.start_state = start_state
        # self.goal_state = goal_state
        self.RPM1 = RPM1
        self.RPM2 = RPM2
        self.actions = np.array([[0, self.RPM1],
                                 [self.RPM1, 0],
                                 [self.RPM1, self.RPM1],
                                 [0, self.RPM2],
                                 [self.RPM2, 0],
                                 [self.RPM2, self.RPM2],
                                 [self.RPM1, self.RPM2],
                                 [self.RPM2, self.RPM1]])
        self.radius = 0.33 # [cm]
        self.L = 1.60 # [cm]

        self.clearance = clearance
        self.threshold = threshold
        self.step_size = step_size
        self.dt = dt
        self.theta_step = to_rad(theta_step)
        self.duplicate_threshold = 0.5
        
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.callback_start_pose)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.callback_goal_pose)
        rospy.Subscriber("map", OccupancyGrid, self.callback_occupancy_grid)

    def callback_start_pose(self, start_pose):
        self.start_pose = start_pose
        yaw = get_yaw(start_pose.pose.pose.orientation)

        self.start_state = (start_pose.pose.pose.position.x,
                            start_pose.pose.pose.position.y,
                            yaw)
        print("Start State: ", self.start_state)
        self.start_node = Node(self.start_state, [self.start_state], None, 0, 0, None)
        
        start_pose_msg = PoseStamped()
        start_pose_msg.header.stamp = rospy.Time.now()
        start_pose_msg.header.frame_id = "map"
        start_pose_msg.pose.position = start_pose.pose.pose.position
        start_pose_msg.pose.orientation = start_pose.pose.pose.orientation
        self.start_pose_pub.publish(start_pose_msg)

    def callback_goal_pose(self, goal_pose):
        print("")
        print("Start State: ", self.start_state)
        self.start_node = Node(self.start_state, [self.start_state], None, 0, 0, None)
        
        start_pose_msg = PoseStamped()
        start_pose_msg.header.stamp = rospy.Time.now()
        start_pose_msg.header.frame_id = "map"
        start_pose_msg.pose.position.x = self.start_state[0]
        start_pose_msg.pose.position.y = self.start_state[1]
        start_pose_msg.pose.position.z = 0
        quat = quaternion_from_euler(0, 0, self.start_state[2])
        start_pose_msg.pose.orientation.x = quat[0]
        start_pose_msg.pose.orientation.y = quat[1]
        start_pose_msg.pose.orientation.z = quat[2]
        start_pose_msg.pose.orientation.w = quat[3]
        self.start_pose_pub.publish(start_pose_msg)

        self.goal_pose = goal_pose
        yaw = get_yaw(goal_pose.pose.orientation)

        self.goal_state = (goal_pose.pose.position.x,
                           goal_pose.pose.position.y,
                           0)
        self.goal_node = Node(self.goal_state, [self.goal_state], None, 0, -1, None)

        print("")
        print("Goal State: ", self.goal_state)
        goal_point = PointStamped()
        goal_point.header.stamp = rospy.Time.now()
        goal_point.header.frame_id = "map"
        goal_point.point.x = self.goal_state[0]
        goal_point.point.y = self.goal_state[1]
        self.goal_point_pub.publish(goal_point)

        if(self.search()):
            path, cmds = self.backtrack_path()
            pp = PurePursuit(path, self.goal_state)
            pp.track_path()

    def callback_occupancy_grid(self, occupancy_grid):
        self.occupancy_grid = np.asarray(occupancy_grid.data, dtype=np.int8).reshape(occupancy_grid.info.height, occupancy_grid.info.width)
        self.xy_resolution = occupancy_grid.info.resolution
        print("\nRecieved the occupancy grid map")

        # visited_map_size = [float(self.occupancy_grid.shape[0])/self.duplicate_threshold, float(self.occupancy_grid.shape[1])/self.duplicate_threshold, 360.0/to_deg(self.theta_step)]
        visited_map_size = [float(self.occupancy_grid.shape[0]), float(self.occupancy_grid.shape[1]), 360.0/to_deg(self.theta_step)]
        self.visited_map = np.array(np.ones(list(map(int, visited_map_size))) * np.inf)
        # print(self.visited_map.shape)

        # if(self.start_state is not None and self.goal_state is not None):
        #     self.simulate()

    def __in_collision(self, pos, clearance):
        x = pos[0]//self.xy_resolution
        y = pos[1]//self.xy_resolution

        X, Y = np.ogrid[max(0, int(x) - clearance):min(self.occupancy_grid.shape[0], int(x) + clearance), max(0, int(y) - clearance):min(self.occupancy_grid.shape[1], int(y) + clearance)]
        X = np.int32(X)
        Y = np.int32(Y)
        # print(X, Y)

        # print("state: ", pos)
        # print("loc: ", x, y)
        # print("collision: ", self.occupancy_grid[int(x),int(y)])
        # print("clearance: ", len(np.where(np.int32(self.occupancy_grid[X, Y] == 0)[0])))
        # print(np.where(np.int32(self.occupancy_grid[X, Y] == 0)[0]))
        # input('q')
        if(x < 0 or x >= self.occupancy_grid.shape[0] or y < 0 or y >= self.occupancy_grid.shape[1]):
            # print("Node out of bounds!")
            return True
        elif(self.occupancy_grid[int(y),int(x)] != 0):
            # print("Node in collision!")
            return True
        elif(len(np.where(self.occupancy_grid[Y, X] != 0)[0])):
            # print(len(np.where(np.int32(self.occupancy_grid[Y, X] != 0)[0])))
            # print("CLEARANCE LIMIT! ")
            return True
        else:
            # print("NO COLLISION!")
            return False

    def __is_visited(self, node):
        x = node.state[0]//self.xy_resolution
        y = node.state[1]//self.xy_resolution
        # x = int((round(node.state[0] * 2) / 2)/self.duplicate_threshold)
        # y = int((round(node.state[1] * 2) / 2)/self.duplicate_threshold)

        theta = round(to_deg(node.state[2]))
        if(theta < 0.0):
            theta += 360
        theta = int(theta/30)

        # print("visited_map: ", x, y, theta)
        if(self.visited_map[int(y)][int(x)][theta] != np.inf):
            # if(self.visited_map[x][y][theta] > node.cost):
            #     self.open_list[node.state] = (node.index, node)
            #     self.open_list[new_state] = (new_node.index, new_node)
            #     self.visited_map[x][y][theta] = node.cost
            return True
        else:
            return False

    def __update_visited(self, node):
        x = int(node.state[0]//self.xy_resolution)
        y = int(node.state[1]//self.xy_resolution)
        # x = int((round(node.state[0] * 2) / 2)/self.duplicate_threshold)
        # y = int((round(node.state[1] * 2) / 2)/self.duplicate_threshold)

        theta = round(to_deg(node.state[2]))
        if(theta < 0.0):
            theta += 360
        theta = int(theta/30)

        # print(x, y, theta)
        self.visited_map[int(y)][int(x)][theta] = node.cost

    def __to_tuple(self, state):
        return tuple(state)

    def __euclidean_distance(self, p1, p2):
        return (np.sqrt(sum((np.asarray(p1) - np.asarray(p2))**2)))

    def __motion_model(self, state, RPM1, RPM2, parent_index, parent_cost):
        u_l = RPM1
        u_r = RPM2
        # u_l = ((2*np.pi*self.radius)*RPM1)/60
        # u_r = ((2*np.pi*self.radius)*RPM2)/60
        multiplier = ((2*np.pi*self.radius)/60)
        # multiplier = 1
        
        x = state[0]
        y = state[1]
        theta = state[2]

        path = list()
        lin_vel = list()
        ang_vel = list()
        for t in range(int(self.step_size/self.dt)):
            theta_dt = ((self.radius/self.L)*(u_l - u_r))
            theta = (theta + (theta_dt*self.dt))
            ang_vel.append(theta_dt*multiplier)
            # ang_vel.append((self.radius/2) * (u_l + u_r) * multiplier)

            x_dt = (0.5*self.radius*(u_l + u_r)*np.cos(state[2]))
            x = x + (x_dt*self.dt)
            x_vel = (0.5*self.radius*(u_l + u_r)*np.cos(theta_dt*multiplier))

            y_dt = (0.5*self.radius*(u_l + u_r)*np.sin(state[2]))
            y = y + (y_dt*self.dt)
            y_vel = (0.5*self.radius*(u_l + u_r)*np.sin(theta_dt*multiplier))
           
            lin_vel.append(np.sqrt((x_vel*multiplier)**2 + (y_vel*multiplier)**2))
            # lin_vel.append((self.radius/self.L) * (u_l - u_r) * multiplier)

            if(not self.__in_collision((x, y, theta), self.clearance)):
                path.append((x, y, theta))
            else:
                return None, False
        # print((0.5*self.radius*(u_l + u_r)*np.cos(state[2])*self.step_size),
        #       (0.5*self.radius*(u_l + u_r)*np.sin(state[2])*self.step_size),
        #       ((self.radius/self.L)*(u_l - u_r)*self.step_size))

        new_state = (x, y, theta)
        new_cost = parent_cost + self.step_size
        new_index = parent_index + 1
        cmds = [lin_vel, ang_vel]

        new_node = Node(new_state, path, cmds, new_cost, new_index, parent_index)
        return new_node, True

    def search(self):

        print("\nStarting search...\n")

        self.current_index = 0
        self.open_list = dict()
        self.closed_list = dict()
        self.final_node = None
        self.path = None
        self.iterations = 0
        self.nodes = 0
        self.search_cost = 0.0
        self.visualize = True

        pq = PriorityQueue()

        pq.put((self.start_node.cost, self.start_node.state))
        self.open_list[self.start_node.state] = (self.start_node.index, self.start_node)

        prev_node = self.start_node

        visited_poses_msg = PoseArray()

        tick = time.time()
        while(not pq.empty()):

            self.iterations += 1

            current_node = self.open_list[pq.get()[1]][1]
            self.__update_visited(current_node)
            self.closed_list[current_node.state] = (current_node.index, current_node)
            del self.open_list[current_node.state]

            # print("Prev State: ", prev_node.state)
            # print("Current State: ", current_node.state)

            if(False):
                visited_poses_msg.header.stamp = rospy.Time.now()
                visited_poses_msg.header.frame_id = "map"

                current_pose_msg = Pose()
                # current_pose_msg.header.stamp = rospy.Time.now()
                # current_pose_msg.header.frame_id = "map"
                position, quat = get_pose(current_node.state)
                # current_pose_msg.pose.position = position
                # current_pose_msg.pose.orientation = quat
                current_pose_msg.position = self.start_pose.pose.position
                current_pose_msg.orientation = self.start_pose.pose.pose.orientation
                # print(current_pose_msg)
                visited_poses_msg.poses.append(current_pose_msg)
                # print(visited_poses_msg)
                # self.current_pose_pub.publish(current_pose_msg)
                self.visited_poses_pub.publish(visited_poses_msg)

            if(self.__euclidean_distance(current_node.state[:2], self.goal_state[:2]) <= self.threshold):
                print("GOAL REACHED!")
                toc = time.time()
                print("Took %.03f seconds to search the path"%((toc-tick)))
                self.final_node = current_node
                self.search_cost = current_node.cost
                return True

            for action in self.actions:
                new_node, valid = self.__motion_model(current_node.state, action[0], action[1], current_node.index, current_node.cost)

                if(valid == True):
                    new_state = new_node.state
                    self.current_index = new_node.index

                    if(self.__is_visited(new_node)):
                        # print("Visited")
                        self.current_index -= 1
                        continue
                    # else:
                        # print("Not visited")

                    # if(new_state in self.closed_list):
                    #     print("Visited")
                    #     self.current_index -= 1
                    #     continue

                    if(self.visualize):
                        visited_poses_msg.header.stamp = rospy.Time.now()
                        visited_poses_msg.header.frame_id = "map"

                        current_pose_msg = Pose()
                        current_pose_msg.position.x = new_state[0]
                        current_pose_msg.position.y = new_state[1]
                        current_pose_msg.position.z = 0
                        quat = quaternion_from_euler(0, 0, new_state[2])
                        current_pose_msg.orientation.x = quat[0]
                        current_pose_msg.orientation.y = quat[1]
                        current_pose_msg.orientation.z = quat[2]
                        current_pose_msg.orientation.w = quat[3]
                        visited_poses_msg.poses.append(current_pose_msg)
                        self.visited_poses_pub.publish(visited_poses_msg)

                    if(new_state not in self.open_list):
                        self.open_list[new_state] = (new_node.index, new_node)
                        pq.put((new_node.cost + self.__euclidean_distance(new_state[:2], self.goal_state[:2]), new_node.state))
                    else:
                        if(self.open_list[new_state][1].cost > new_node.cost):
                            self.open_list[new_state] = (new_node.index, new_node)
                        else:
                            self.current_index -= 1

                    self.nodes += 1
                else:
                    self.current_index -= 1
                    # print("NODE IN COLLISION!")
                    pass

            prev_node = current_node

        print("SOLUTION DOES NOT EXIST!")
        return False

    def backtrack_path(self):

        current_node = self.final_node
        self.path = list()
        self.cmds = list()
        closed_list = dict(self.closed_list.values())

        print("BACKTRACKING PATH...")

        while(current_node.index != 0):
            lin_vel = list()
            ang_vel = list()
            for i in range(len(current_node.path)-1, -1, -1):
                self.path.append(current_node.path[i])
                lin_vel.append(current_node.cmds[0][i])
                ang_vel.append(current_node.cmds[1][i])
            self.cmds.append([lin_vel, ang_vel])
            current_node = closed_list[current_node.parent_index]

        self.path.append(self.start_node.state)
        self.path.reverse()

        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        i=0
        for pose in self.path:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = pose[0]
            pose_msg.pose.position.y = pose[1]
            pose_msg.pose.position.z = 0
            quat = quaternion_from_euler(0, 0, pose[2])
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            # print(pose_msg)
            path.poses.append(pose_msg)
            i+=1

        self.path_pub.publish(path)

        self.path = np.array(self.path).astype(int)
        
        print("BACKTRACKING PATH COMPLETE!")
        print("A* Path Length: {}".format(self.search_cost))
        return path, self.cmds

    def simulate(self):
        print("Simulating A* for Turtlebot 3...")

        print("")
        print("Start State: ", self.start_state)
        self.start_node = Node(self.start_state, [self.start_state], None, 0, 0, None)

        start_pose_msg = PoseStamped()
        start_pose_msg.header.stamp = rospy.Time.now()
        start_pose_msg.header.frame_id = "map"
        start_pose_msg.pose.position.x = self.start_state[0]
        start_pose_msg.pose.position.y = self.start_state[1]
        start_pose_msg.pose.position.z = 0
        quat = quaternion_from_euler(0, 0, self.start_state[2])
        start_pose_msg.pose.orientation.x = quat[0]
        start_pose_msg.pose.orientation.y = quat[1]
        start_pose_msg.pose.orientation.z = quat[2]
        start_pose_msg.pose.orientation.w = quat[3]
        self.start_pose_pub.publish(start_pose_msg)

        print("")
        print("Goal State: ", self.goal_state)
        self.goal_node = Node(self.goal_state, [self.goal_state], None, 0, -1, None)

        goal_point = PointStamped()
        goal_point.header.stamp = rospy.Time.now()
        goal_point.header.frame_id = "map"
        goal_point.point.x = self.goal_state[0]
        goal_point.point.y = self.goal_state[1]
        self.goal_point_pub.publish(goal_point)

        if(self.search()):
            path, cmds = self.backtrack_path()
            pp = PurePursuit(path, self.goal_state)
            rospy.sleep(5)
            pp.track_path()