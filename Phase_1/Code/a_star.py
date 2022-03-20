#!/usr/env/bin python3

"""
ENPM661 Spring 2022: Planning for Autonomous Robots
Project 2: Dijkstra

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

class Node():
    '''
        Class to represent nodes in Breadth First Search

        Attributes
        state: state of the node
        index: index of the node
        parent_index: index of parent node
        actions: Possible actions to generate child nodes
    '''

    def __init__(self, state: tuple, cost: float, index: int, parent_index: int) -> None:
        self.state = state
        self.cost = cost
        self.index = index
        self.parent_index = parent_index


def main():

    Parser = argparse.ArgumentParser()
    Parser.add_argument('--StartState', type=str, default="[10, 10]", help='Start state of the robot')
    Parser.add_argument('--GoalState', type=str, default="[390, 215]", help='Goal state of the robot')
    Parser.add_argument('--Random', action='store_true', help='Toggle randomized start and goal states')
    Parser.add_argument('--Visualize', action='store_true', help='Toggle search visualization')

    Args = Parser.parse_args()
    StartState = tuple(map(int, Args.StartState.replace('[', ' ').replace(']', ' ').replace(',', ' ').split()))
    GoalState = tuple(map(int, Args.GoalState.replace('[', ' ').replace(']', ' ').replace(',', ' ').split()))
    Random = Args.Random
    Visualize = Args.Visualize

    m = Map(400,250)
    m.generate_map()

    D = Dijkstra(StartState, GoalState, m.occupancy_grid, 5, Visualize)
    if(D.valid):
        if(D.search()):
            D.backtrack_path()

if __name__ == '__main__':
    main()