#!/usr/bin/env python3

"""
ENPM661 Spring 2022: Planning for Autonomous Robots
Project 3 - Phase 2: A* for Turtlebot

Author(s):
Tanuj Thakkar (tanuj@umd.edu)
M. Engg Robotics
University of Maryland, College Park
"""

# Importing Modules
import sys
sys.path.append('/usr/lib/python2.7/dist-packages')
# sys.path.remove()
import rospy
import time
import math

from astar import *


def main():
	rospy.init_node('astar_turtlebot', anonymous=True)

	astr = AStar(20, 40, 1.0, 1.5, 0.5, 30.0)
	rospy.spin()

if __name__ == '__main__':
	main()