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
import math
from queue import PriorityQueue
import matplotlib.pyplot as plt


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