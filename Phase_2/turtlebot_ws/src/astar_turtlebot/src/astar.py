
# Importing Module
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import csv
import Queue

from Obstacle import *
from Map import *
from MathUtils import *
from Node import *


class AStar:

    def __init__(self, start_state, goal_state, RPM1, RPM2, clearance, wheel_radius = 0.038, wheel_distance = 0.354, h = 10, w = 10):
        self.start_state = start_state
        self.goal_state = goal_state
        self.RPM1 = RPM1
        self.RPM2 = RPM2
        self.clearance = clearance
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance
        self.h = h
        self.w = w

    def check_reached(self, current_node, goal_state, thresh_radius):
        current_state = current_node.get_state()
        radius_sq = np.square(current_state[0] - goal_state[0]) + np.square(current_state[1] - goal_state[1])
        if radius_sq < thresh_radius**2:
            return True
        else:
            return False

    def estimate_heuristic(self, current_state, goal_state):
        cost = 0.0
        if current_state is not None:
            cost =  ((current_state[0]-goal_state[0])**2 + (current_state[1]-goal_state[1])**2)**(0.5)
        return cost

    def is_visited(self, node, node_array, goal_state, threshold=0.5):
        result = False
        node_state = node.get_state()
        x = node_state[0]
        y = node_state[1]
        theta = node_state[2]
        x = int(half_round(x)/threshold)
        y = int(half_round(y)/threshold)

        if (node.get_cost() + self.estimate_heuristic(node_state, goal_state) < node_array[x, y, theta]):
            result = True
        return result

    def get_child(self, node, T, RPM1, RPM2, obs):
        actions = np.array([[0, RPM1],
                            [RPM1, 0],
                            [RPM1, RPM1],
                            [0, RPM2],
                            [RPM2, 0],
                            [RPM2, RPM2],
                            [RPM1, RPM2],
                            [RPM2, RPM1]])

        state = node.get_state()
        branches = list()
        for action in actions:
            new_state, path_array, cost = self.move(state, action, T, obs)
            if new_state is not None:
                branch_node = Node(new_state, node, action, node.get_cost() + cost, path_array)
                branches.append(branch_node)

        return branches

    def move(self, state, action, T, obs):
        t = 0
        dt = 0.1

        Xi, Yi, thetai = state
        thetai = to_rad(thetai)

        wL, wR = action
        Xn = Xi
        Yn = Yi
        thetan = thetai

        path_array = list()
        cost = 0.0
        path_array.append([Xn, Yn])
        while t<T:
            t = t + dt
            dx = 0.5 * self.wheel_radius * (wL + wR) * math.cos(thetan) * dt
            dy = 0.5 * self.wheel_radius * (wL + wR) * math.sin(thetan) * dt
            Xn += dx
            Yn += dy
            thetan += (self.wheel_radius / self.wheel_distance) * (wL - wR) * dt
            cost += math.sqrt(math.pow(dx,2) + math.pow(dy,2))
            path_array.append([Xn, Yn])

            if obs.in_collision(Xn, Yn):
                return None, None, None

        thetan = int(to_deg(thetan))
        if (thetan >= 360):
            thetan -= 360
        if (thetan <= -360):
            thetan += 360
        return [Xn, Yn, thetan] , path_array, cost

    def search(self):

        h,w = self.h, self.w
        threshold = 0.5
        threshold_angle = 360
        start_state = self.start_state
        goal_state = self.goal_state
        RPM1, RPM2 = self.RPM1, self.RPM2

        nodes = Queue.PriorityQueue()

        init_node = Node(start_state, None, None, 0, None)
        nodes.put((init_node.get_cost(), init_node))
        traversed_nodes = list()
        final_moves = list()
        final_points = list()
        final_path = list()

        obs = Obstacle(self.clearance)
        m = Map(obs)

        fig, ax = plt.subplots(figsize=(10,10))
        ax.set(xlim=(0,10), ylim=(0,10))
        ax = m.generate_map(ax)
        ax.set_aspect("equal")

        goal_reached = False
        node_array = np.array([[[np.inf for k in range(threshold_angle)] for j in range(int(h/threshold))] for i in range(int(w/threshold))])

        full_path = None
        goal_reached = False

        print("\nStarting search...\n")

        while (not nodes.empty()):
            current_node = nodes.get()[1]
            traversed_nodes.append(current_node)

            if(self.check_reached(current_node, goal_state,1)):
                print('Goal reached!')
                print("Path Cost: ", current_node.get_cost())
                moves, node_path = current_node.get_path()
                final_moves = moves
                goal_reached = True

                for node in node_path:
                    xi, yi, _ = node.get_state()
                    final_path.append([xi, yi])
                    points = node.get_path_array()
                    if points is not None:
                        for point in points:
                            xn, yn = point
                            row = [xn, yn]
                            final_points.append(row)
                            xi, yi = xn, yn
            else:
                branches = self.get_child(current_node, 1, RPM1, RPM2, obs)
                for branch_node in branches:
                    branch_state = branch_node.get_state()
                    if self.is_visited(branch_node, node_array, goal_state, threshold=0.5):
                        node_array[int(half_round(branch_state[0])/threshold), int(half_round(branch_state[1])/threshold), branch_state[2]] = branch_node.get_cost() + self.estimate_heuristic(branch_state, goal_state)
                        nodes.put((branch_node.get_cost() + self.estimate_heuristic(branch_state, goal_state), branch_node))
            if(goal_reached):
                break

        return final_moves, final_path, final_points

if __name__ == "__main__":
    astar = AStar([1,1,0], [9,9], 5, 10, 0.1)
    final_moves, final_path, final_points = astar.search()