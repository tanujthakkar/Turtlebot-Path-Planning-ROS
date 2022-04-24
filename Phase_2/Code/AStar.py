#!/usr/env/bin python3

"""
ENPM661 Spring 2022: Planning for Autonomous Robots
Project 3 - Phase 1: A*

"""

# Importing Modules
import sys
import os
import numpy as np
import argparse
import time
import math
from queue import PriorityQueue
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from Map import *
from utils import *

sys.dont_write_bytecode = True

class Node():
    '''
        Class to represent nodes in graph search

        Attributes
        state: state of the node
        cost: cost of the node
        index: index of the node
        parent_index: index of parent node
    '''

    def __init__(self, state: tuple, cost: float, index: int, parent_index: int) -> None:
        self.state = state
        self.cost = cost
        self.index = index
        self.parent_index = parent_index

class AStar:
    # start_state, goal_state, m.occupancy_grid, clearance + radius, threshold, step_size, steer_limit, steer_step, visualize

    def __init__(self, start_state, goal_state, clearance, threshold, step_size, theta_step, RPM1, RPM2 , visualize,m,ax) -> None:
        self.valid = True
        self.start_state = start_state
        self.goal_state = goal_state
   
     
        self.clearance = clearance
        self.threshold = threshold
        self.step_size = step_size
        self.theta_step = to_rad(theta_step)
        self.duplicate_threshold = 0.5

        self.current_index = 0
        self.start_node = Node(self.start_state, 0, self.current_index, None)
        self.goal_node = Node(self.goal_state, 0, -1, None)
        # self.steering_set = np.linspace(-self.steer_limit, self.steer_limit, int((self.steer_limit*2//self.steer_step)+1))
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
        self.radius= 0.033
        self.wheeldistance= 0.160
        self.dt = 0.1
        self.open_list = dict()
        self.closed_list = dict()
        # visited_map_size = [float(100)/self.duplicate_threshold, float(100)/self.duplicate_threshold, 360.0/to_deg(self.theta_step)]
        self.visited_map = np.ones((10 * 2, 10 * 2, 360 // 30), np.int32) * math.inf
        self.final_node = None
        self.path = None
        self.visualize = visualize
        self.iterations = 0
        self.nodes = 0
        self.search_cost = 0.0
        self.occupancy_grid_ = None
        self.map_class = m
        self.ax = ax

        if self.map_class.check_collision(self.start_node.state[0],self.start_node.state[1]):
            print("invalid start")
        # if(self.visualize):
            # self.video = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc('F','M','P','4'), 24, (self.occupancy_grid.shape[0], self.occupancy_grid.shape[1]))

        print("\nInitialized A*...\n")
        print("Initial State: \n", self.start_node.state)
        print("Goal State: \n", self.goal_node.state)


    def __in_collision(self, pos, clearance, map_class) -> bool:
        if map_class.check_collision(pos[0],pos[1]):
            return True
        else:
            return False


    def __update_visited(self, node: Node) -> None:
        x = int((round(node.state[0] * 2) / 2)/self.duplicate_threshold)
        y = int((round(node.state[1] * 2) / 2)/self.duplicate_threshold)
        theta = round(to_deg(node.state[2]))
        if(theta < 0.0):
            theta += 360
        theta = int(theta/30)

        self.visited_map[x][y][theta] = node.cost

    def __to_tuple(self, state: np.array) -> tuple:
        return tuple(state)

    def __euclidean_distance(self, p1: tuple, p2: tuple) -> float:
    	return (np.sqrt(sum((np.asarray(p1) - np.asarray(p2))**2)))

    def __motion_model(self, state: tuple, left: int,right: int) -> tuple:
        r, L, dt = self.radius, self.wheeldistance, self.dt
        u_l = left
        u_r = right

        x = state[0] + (0.5*self.radius*(u_l + u_r)*np.cos(state[2])*self.step_size)
        y = state[1] + (0.5*self.radius*(u_l + u_r)*np.sin(state[2])*self.step_size)
        theta = pi_2_pi(state[2] + ((self.radius/self.wheeldistance)*(u_l - u_r)*self.theta_step))

        return (x, y, theta)

    def search(self) -> bool:

        print("\nStarting search...\n")

        pq = PriorityQueue()

        pq.put((self.start_node.cost, self.start_node.state))
        self.open_list[self.start_node.state] = (self.start_node.index, self.start_node)

        prev_node = self.start_node

        if(self.visualize):
            print("plotting start and end position")
            self.ax.add_patch(patches.Circle( (self.start_state[0],self.start_state[1]), radius=0.09, linewidth=1, alpha=0.5, edgecolor='r', facecolor='r'))
            self.ax.add_patch(patches.Circle( (self.goal_state[0],self.goal_state[1]), radius=0.09, linewidth=1, alpha=0.5, edgecolor='g', facecolor='g'))
            

        tick = time.time()
        while(not pq.empty()):

            self.iterations += 1

            current_node = self.open_list[pq.get()[1]][1]
            self.__update_visited(current_node)
            self.closed_list[current_node.state] = (current_node.index, current_node)
            del self.open_list[current_node.state]

          
            if(self.visualize):
                try:
                    closed_list_ = dict(self.closed_list.values())
                    parent_node = closed_list_[current_node.parent_index]

                    start = (int(parent_node.state[0]), int(parent_node.state[1]))
                    end = (int(current_node.state[0]), int(current_node.state[1]))
                    self.ax.plot([start[0], end[0]], [start[1], end[1]], color='r', linewidth = 2)
            
                except Exception as e:
                    print(e)
                    pass

            if(self.__euclidean_distance(current_node.state[:2], self.goal_state[:2]) <= self.threshold):
                print("GOAL REACHED!")
                toc = time.time()
                # print("Took %.03f seconds to search the path"%((toc-tick)))
                self.final_node = current_node
                
                self.search_cost = current_node.cost
                return True,self.ax

            for action in self.actions:
                new_state = self.__motion_model(current_node.state, action[0],action[1])
                new_index = self.current_index + 1
                self.current_index = new_index
                new_cost = current_node.cost + self.step_size

                if not (self.__in_collision(new_state, self.clearance,self.map_class)):
                    new_node = Node(new_state, new_cost, new_index, current_node.index)


                    if(new_state in self.closed_list):
                        self.current_index -= 1
                        continue

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
        return False,self.ax

    def backtrack_path(self,ax) -> np.array:

        current_node = self.final_node
        self.path = list()
        closed_list = dict(self.closed_list.values())

        print("BACKTRACKING PATH...")

        while(current_node.index != 0):
            self.path.append(current_node.state)
            current_node = closed_list[current_node.parent_index]

        self.path.append(self.start_node.state)
        self.path.reverse()
        self.path = np.array(self.path).astype(int)

      
        
        if(self.visualize):
            for step in range(len(self.path)-1):
                start = (self.path[step,0], self.path[step,1])
                end = (self.path[step+1,0], self.path[step+1,1])
                self.ax.plot([start[0], end[0]], [start[1], end[1]], color='g', linewidth = 2)
           
           
        print("BACKTRACKING PATH COMPLETE!")
        print("A* Path Length: {}".format(self.search_cost))
        return self.path,ax


def main():

    Parser = argparse.ArgumentParser()
    Parser.add_argument('--StartState', type=str, default="[15, 15, 0]", help='Start state of the robot')
    Parser.add_argument('--GoalState', type=str, default="[50, 50, 0]", help='Goal state of the robot')
    Parser.add_argument('--Radius', type=int, default=10, help='Radius of the robot')
    Parser.add_argument('--Clearance', type=int, default=5, help='Clearance to obstacles')
    Parser.add_argument('--Threshold', type=float, default=1.5, help='Threshold to goal')
    Parser.add_argument('--StepSize', type=float, default=1.0, help='Step size for the search')
    Parser.add_argument('--RPM1', type=float, default=25, help='Steering limit in either right/left direction')
    Parser.add_argument('--RPM2', type=float, default=15, help='Steering step')
    Parser.add_argument('--ThetaStep', type=float, default=30, help='Steering step')
    Parser.add_argument('--Random', action='store_true', help='Toggle randomized start and goal states')
    Parser.add_argument('--Visualize', action='store_true', help='Toggle search visualization')

    Args = Parser.parse_args()
    start_state = list(map(int, Args.StartState.replace('[', ' ').replace(']', ' ').replace(',', ' ').split()))
    start_state[2] = to_rad(start_state[2])
    start_state = tuple(start_state)
    goal_state = list(map(int, Args.GoalState.replace('[', ' ').replace(']', ' ').replace(',', ' ').split()))
    goal_state[2] = to_rad(goal_state[2])
    goal_state = tuple(goal_state)
    radius = Args.Radius
    clearance = Args.Clearance
    threshold = Args.Threshold
    step_size = Args.StepSize
    RPM1 = Args.RPM1
    RPM2 = Args.RPM2
    theta_step = Args.ThetaStep
    random = Args.Random
    visualize = Args.Visualize

    fig, ax = plt.subplots()
    ax.set(xlim=(0, 10), ylim = (0,10))
    m = Map(10,10,0.1)
    ax= m.generate_map(ax)
    ax.set_aspect("equal")
    
    astar = AStar(start_state, goal_state, clearance, threshold, step_size, theta_step, RPM1, RPM2 , visualize,m,ax)
    if(astar.valid):
        val , ax = astar.search()
        if(val):
            path, ax =astar.backtrack_path(ax)
            plt.show()
if __name__ == '__main__':
    main()

