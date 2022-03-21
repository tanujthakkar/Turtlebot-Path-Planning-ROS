#!/usr/env/bin python3

"""
ENPM661 Spring 2022: Planning for Autonomous Robots
Project 3 - Phase 1: A*

Author(s):
Tanuj Thakkar (tanuj@umd.edu)
M. Engg Robotics
University of Maryland, College Park
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

    def __init__(self, start_state: tuple, goal_state: tuple, occupancy_grid:  np.array, clearance: int = 0, threshold: float = 1.5, step_size: float = 1.0, steer_limit: float = 60.0, steer_step: float = 30.0, visualize: bool = False) -> None:
        self.valid = True
        self.start_state = start_state
        self.goal_state = goal_state
        self.occupancy_grid = occupancy_grid
        if(self.__in_collision(start_state, clearance)):
            print("INVALID START STATE!")
            self.valid = False
            return
        if(self.__in_collision(goal_state, clearance)):
            print("INVALID GOAL STATE!")
            self.valid = False
            return
        self.clearance = clearance
        self.threshold = threshold
        self.step_size = step_size
        self.steer_limit = to_rad(steer_limit)
        self.steer_step = to_rad(steer_step)
        self.duplicate_threshold = 0.5

        self.current_index = 0
        self.start_node = Node(self.start_state, 0, self.current_index, None)
        self.goal_node = Node(self.goal_state, 0, -1, None)
        self.steering_set = np.linspace(-self.steer_limit, self.steer_limit, int((self.steer_limit*2//self.steer_step)+1))
        self.open_list = dict()
        self.closed_list = dict()
        visited_map_size = [float(self.occupancy_grid.shape[0])/self.duplicate_threshold, float(self.occupancy_grid.shape[1])/self.duplicate_threshold, 360.0/to_deg(self.steer_step)]
        self.visited_map = np.array(np.ones(list(map(int, visited_map_size))) * np.inf)
        self.final_node = None
        self.path = None
        self.visualize = visualize
        self.iterations = 0
        self.nodes = 0
        self.search_cost = 0.0
        self.occupancy_grid_ = None

        if(self.visualize):
            self.video = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc('F','M','P','4'), 24, (self.occupancy_grid.shape[0], self.occupancy_grid.shape[1]))

        print("\nInitialized A*...\n")
        print("Initial State: \n", self.start_node.state)
        print("Goal State: \n", self.goal_node.state)


    def __in_collision(self, pos: np.array, clearance: int) -> bool:
        X, Y = np.ogrid[max(0, int(pos[0]) - clearance):min(self.occupancy_grid.shape[0], int(pos[0]) + clearance), max(0, int(pos[1]) - clearance):min(self.occupancy_grid.shape[1], int(pos[1]) + clearance)]
        if(pos[0] < 0 or pos[0] >= self.occupancy_grid.shape[0] or pos[1] < 0 or pos[1] >= self.occupancy_grid.shape[1]):
            # print("Node out of bounds!")
            return True
        elif(not self.occupancy_grid[int(pos[0]),int(pos[1])]):
            # print("Node in collision!")
            return True
        elif(len(np.where(self.occupancy_grid[X, Y] == 0)[0])):
            return True
        else:
            return False

    def __is_visited(self, state: tuple):
        x = int((round(state[0] * 2) / 2)/self.duplicate_threshold)
        y = int((round(state[1] * 2) / 2)/self.duplicate_threshold)
        theta = round(to_deg(state[2]))
        if(theta < 0.0):
            theta += 360
        theta = int(theta/30)

        if(self.visited_map[x][y][theta] != np.inf):
            return True
        else:
            return False

    def __to_tuple(self, state: np.array) -> tuple:
        return tuple(state)

    def __euclidean_distance(self, p1: tuple, p2: tuple) -> float:
    	return (np.sqrt(sum((np.asarray(p1) - np.asarray(p2))**2)))

    def __motion_model(self, state: tuple, steering_input: int) -> tuple:
        x = state[0] + (self.step_size * np.cos(steering_input))
        y = state[1] + (self.step_size * np.sin(steering_input))
        theta = pi_2_pi(state[2] + steering_input)

        return (x, y, theta)

    def search(self) -> bool:

        print("\nStarting search...\n")

        pq = PriorityQueue()

        pq.put((self.start_node.cost, self.start_node.state))
        self.open_list[self.start_node.state] = (self.start_node.index, self.start_node)

        prev_node = self.start_node

        if(self.visualize):
            occupancy_grid = np.uint8(np.copy(self.occupancy_grid))
            occupancy_grid = cv2.cvtColor(np.flip(np.uint8(occupancy_grid).transpose(), axis=0), cv2.COLOR_GRAY2BGR)
            cv2.circle(occupancy_grid, (self.start_state[0], self.occupancy_grid.shape[1] - self.start_state[1]), 2, (0, 255, 0), 2)
            cv2.circle(occupancy_grid, (self.goal_state[0], self.occupancy_grid.shape[1] - self.goal_state[1]), 2, (0, 0, 255), 2)
            self.video.write(np.uint8(occupancy_grid))
            cv2.namedWindow("A*", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("A*", 800, 500)
            cv2.imshow("A*", occupancy_grid)
            cv2.waitKey(0)

        tick = time.time()
        while(not pq.empty()):

            self.iterations += 1

            current_node = self.open_list[pq.get()[1]][1]
            self.closed_list[current_node.state] = (current_node.index, current_node)
            del self.open_list[current_node.state]

            # print("Prev State: ", prev_node.state)
            # print("Current State: ", current_node.state)

            if(self.visualize):
                start = (int(prev_node.state[0]), (self.occupancy_grid.shape[1] - 1) - int(prev_node.state[1]))
                end = (int(current_node.state[0]), (self.occupancy_grid.shape[1] - 1) - int(current_node.state[1]))
                cv2.line(occupancy_grid, start, end, (255,0,0), 2)
                # occupancy_grid[row, int(current_node.state[0])] = (242, 133, 65)
                if(self.iterations%20 == 0):
                    self.video.write(np.uint8(occupancy_grid))
                    cv2.imshow("A*", occupancy_grid)
                    cv2.waitKey(3)

            if(self.__euclidean_distance(current_node.state[:2], self.goal_state[:2]) <= self.threshold):
                print("GOAL REACHED!")
                toc = time.time()
                # print("Took %.03f seconds to search the path"%((toc-tick)))
                self.final_node = current_node
                if(self.visualize):
                    self.occupancy_grid_ = occupancy_grid
                self.search_cost = current_node.cost
                return True

            for steering in self.steering_set:
                new_state = self.__motion_model(current_node.state, steering)
                new_index = self.current_index + 1
                self.current_index = new_index
                new_cost = current_node.cost + self.step_size

                if(not self.__in_collision(new_state, self.clearance)):
                    new_node = Node(new_state, new_cost, new_index, current_node.index)

                    if(self.__is_visited(new_state)):
                        self.current_index -= 1
                        continue

                    # if(new_state in self.closed_list):
                    #     self.current_index -= 1
                    #     continue

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

        plt.show()
        print("SOLUTION DOES NOT EXIST!")
        return False

    def backtrack_path(self) -> np.array:

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

        self.occupancy_grid_ = cv2.circle(self.occupancy_grid_, (self.start_state[0], self.occupancy_grid.shape[1] - self.start_state[1]), 2, (0, 255, 0), 2)
        self.occupancy_grid_ = cv2.circle(self.occupancy_grid_, (self.goal_state[0], self.occupancy_grid.shape[1] - self.goal_state[1]), 2, (0, 0, 255), 2)
        
        if(self.visualize):
            for step in range(len(self.path)-1):
                self.occupancy_grid_ = cv2.line(self.occupancy_grid_, (self.path[step,0], self.occupancy_grid.shape[1] - self.path[step,1]), (self.path[step+1,0], self.occupancy_grid.shape[1] - self.path[step+1,1]), (0,0,255), 2)
                cv2.imshow("A*", self.occupancy_grid_)
                self.video.write(np.uint8(self.occupancy_grid_))
                cv2.waitKey(1)
            cv2.waitKey(0)

            cv2.destroyAllWindows()
            self.video.release()

        print("BACKTRACKING PATH COMPLETE!")
        print("A* Path Length: {}".format(self.search_cost))
        return self.path


def main():

    Parser = argparse.ArgumentParser()
    Parser.add_argument('--StartState', type=str, default="[10, 10, 0]", help='Start state of the robot')
    Parser.add_argument('--GoalState', type=str, default="[50, 50, 0]", help='Goal state of the robot')
    Parser.add_argument('--Radius', type=int, default=10, help='Radius of the robot')
    Parser.add_argument('--Clearance', type=int, default=5, help='Clearance to obstacles')
    Parser.add_argument('--Threshold', type=float, default=1.5, help='Threshold to goal')
    Parser.add_argument('--StepSize', type=float, default=1.0, help='Step size for the search')
    Parser.add_argument('--SteerLimit', type=float, default=60.0, help='Steering limit in either right/left direction')
    Parser.add_argument('--SteerStep', type=float, default=30.0, help='Steering step')
    Parser.add_argument('--Random', action='store_true', help='Toggle randomized start and goal states')
    Parser.add_argument('--Visualize', action='store_true', help='Toggle search visualization')

    Args = Parser.parse_args()
    start_state = tuple(map(int, Args.StartState.replace('[', ' ').replace(']', ' ').replace(',', ' ').split()))
    goal_state = tuple(map(int, Args.GoalState.replace('[', ' ').replace(']', ' ').replace(',', ' ').split()))
    radius = Args.Radius
    clearance = Args.Clearance
    threshold = Args.Threshold
    step_size = Args.StepSize
    steer_limit = Args.SteerLimit
    steer_step = Args.SteerStep
    random = Args.Random
    visualize = Args.Visualize

    m = Map(400,250)
    m.generate_map()

    astar = AStar(start_state, goal_state, m.occupancy_grid, clearance + radius, threshold, step_size, steer_limit, steer_step, visualize)
    if(astar.valid):
        if(astar.search()):
            astar.backtrack_path()

if __name__ == '__main__':
    main()