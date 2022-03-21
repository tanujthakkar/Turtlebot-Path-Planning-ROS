#!/usr/env/bin python3

"""
ENPM661 Spring 2022: Planning for Autonomous Robots
Project 2: Dijkstra

Author(s):
Tanuj Thakkar (tanuj@umd.edu)
M. Engg Robotics
University of Maryland, College Park
"""

# Importing modules
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import cv2

sys.dont_write_bytecode = True

class Map():

    def __init__(self, height: int, width: int) -> None:
        self.height = height
        self.width = width
        self.xx, self.yy = np.mgrid[0:height,0:width]
        self.occupancy_grid = np.zeros([self.height, self.width], dtype=int)

    def generate_map(self) -> None:
        c = Circle(300, 185, 40)
        circle = c.get_points_inside(self.xx, self.yy, 0)
        self.occupancy_grid += circle

        h = Hexagon([(165,120.20), (200,140.41), (235,120.20), (235,79.8), (200,59.58), (165,79.8), (165,120.20)])
        hexagon = h.get_points_inside(self.xx, self.yy, 0)
        self.occupancy_grid += hexagon        

        t1 = Triangle1([(36,185), (105,100), (80,180), (36,185)])
        t1_ = t1.get_points_inside(self.xx, self.yy, 0)
        self.occupancy_grid += t1_

        t2 = Triangle2([(36,185), (80,180), (115,210), (36,185)])
        t2_ = t2.get_points_inside(self.xx, self.yy, 0)
        self.occupancy_grid += t2_

        self.occupancy_grid[self.occupancy_grid==0] = 255 # Free Space
        self.occupancy_grid[self.occupancy_grid==1] = 0 # Obstacles

        # cv2.imshow("", np.flip(np.uint8(self.occupancy_grid).transpose(), axis=0))
        # cv2.waitKey(0)


class Circle():

    def __init__(self, x: int, y: int, radius: int) -> None:
        self.name = "Circle"
        self.x = x
        self.y = y
        self.radius = radius

    def __str__(self) -> str:
        return "Circle of radius {} with center ({}, {})".format(self.radius, self.x, self.y)

    def get_points_inside(self, xx: np.array, yy: np.array, clearance: int) -> np.array:
        occupancy_grid = np.logical_and(True, (((xx - self.x) ** 2 + (yy - self.y) ** 2) <= (self.radius + clearance)**2))
        occupancy_grid[occupancy_grid == True] = 1

        return occupancy_grid


class Line():

    def __init__(self, p1: tuple, p2: tuple) -> None:
        self.p1 = p1
        self.p2 = p2

    def __str__(self) -> str:
        return "Line defined by point ({}, {}) and ({}, {})".format(self.p1[0], self.p1[1], self.p2[0], self.p2[1])

    def get_line(self) -> np.array:
        # Equation of line: y = mx + c
        X = np.array([[self.p1[0], 1],
                      [self.p2[0], 1]])
        Y = np.array([[self.p1[1]], [self.p2[1]]])
        a = np.empty([2,1])

        a = np.matmul(np.matmul(np.linalg.pinv(np.matmul(X.transpose(), X)), X.transpose()), Y)

        return a


class Triangle1():

    def __init__(self, points: list) -> None:
        self.points = points
        self.m = list()
        self.c = list()

    def get_lines(self) -> None:
        for line in range(len(self.points)-1):
            l = Line(self.points[line], self.points[line+1])
            a = l.get_line()
            self.m.append(a[0])
            self.c.append(a[1])

    def get_points_inside(self, xx: np.array, yy: np.array, clearance: int) -> np.array:
        self.get_lines()
        t1 = np.logical_and((yy + clearance >= ((self.m[0]*xx) + self.c[0])), (yy + clearance <= ((self.m[1]*xx) + self.c[1])))
        t2 = np.logical_and(t1, (yy + clearance <= ((self.m[2]*xx) + self.c[2])))
        occupancy_grid = np.logical_and(t1, t2)
        occupancy_grid[occupancy_grid == True] = 1

        return occupancy_grid


class Triangle2():

    def __init__(self, points: list) -> None:
        self.points = points
        self.m = list()
        self.c = list()

    def get_lines(self) -> None:
        for line in range(len(self.points)-1):
            l = Line(self.points[line], self.points[line+1])
            a = l.get_line()
            self.m.append(a[0])
            self.c.append(a[1])

    def get_points_inside(self, xx: np.array, yy: np.array, clearance: int) -> np.array:
        self.get_lines()
        t1 = np.logical_and((yy + clearance >= ((self.m[0]*xx) + self.c[0])), (yy + clearance >= ((self.m[1]*xx) + self.c[1])))
        t2 = np.logical_and(t1, (yy + clearance <= ((self.m[2]*xx) + self.c[2])))
        occupancy_grid = np.logical_and(t1, t2)
        occupancy_grid[occupancy_grid == True] = 1

        return occupancy_grid


class Hexagon():

    def __init__(self, points: list) -> None:
        self.points = points
        self.m = list()
        self.c = list()

    def get_lines(self) -> None:
        for line in range(len(self.points)-1):
            l = Line(self.points[line], self.points[line+1])
            a = l.get_line()
            self.m.append(a[0])
            self.c.append(a[1])

    def get_points_inside(self, xx: np.array, yy: np.array, clearance: int) -> np.array:
        self.get_lines()
        l = np.logical_and((yy + clearance <= ((self.m[0]*xx) + self.c[0])), (yy + clearance <= ((self.m[1]*xx) + self.c[1])))
        l = np.logical_and(l, (xx + clearance >= 165))
        l = np.logical_and(l, (yy + clearance >= ((self.m[3]*xx) + self.c[3])))
        l = np.logical_and(l, (yy + clearance >= ((self.m[4]*xx) + self.c[4])))
        l = np.logical_and(l, (xx + clearance <= 235))
        occupancy_grid = np.logical_and(True, l)
        occupancy_grid[occupancy_grid == True] = 1

        return occupancy_grid


def main():
    m = Map(400,250)
    m.generate_map()

if __name__ == '__main__':
    main()