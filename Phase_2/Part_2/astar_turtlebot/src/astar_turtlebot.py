#!/usr/bin/env python2

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

    if(len(sys.argv) > 1):
        start_state = (float(sys.argv[1])+5.25, float(sys.argv[2])+5.25, float(sys.argv[3]))
        goal_state = None
        RPM1 = int(sys.argv[4])
        RPM2 = int(sys.argv[5])
        clearance = int(sys.argv[6])
    else:
        start_state = None
        goal_state = None
        RPM1 = 10
        RPM2 = 15
        clearance = 5

    astr = AStar(start_state, RPM1, RPM2, clearance, 0.5, 0.1, 0.1, 30.0) # [RPM1, RPM2, clearance, threshold, step_size, dt, theta_step]
    rospy.spin()

if __name__ == '__main__':
    main()