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
import matplotlib.patches as patches
sys.dont_write_bytecode = True

class Map():

    def __init__(self, height: int, width: int,clearance: int) -> None:
        self.height = height
        self.width = width
        self.clearance = clearance
        self.obstacles = []

    def generate_map(self,ax) -> None:
        circle1 = Circle(2,2,1,self.clearance)
        square = Rectangle(1,5,1.5,1.5,self.clearance)
        circle2 = Circle(2,self.height -2,1,self.clearance)
        reactangle1 = Rectangle(5,5,2.5,1.5,self.clearance)
        reactangle2 = Rectangle(8,3,1.5,2,self.clearance)

        self.obstacles = [circle1,square,circle2,reactangle1,reactangle2]
        for obstacle in self.obstacles:
            obstacle.add_plot(ax)
        return ax

    def check_collision(self,x,y):
        if not ((self.clearance <x< self.width-self.clearance) and  (self.clearance < y < self.height-self.clearance)):
            collision = True
            print("Point in map")
        #for obstacles
        circle1,square,circle2,reactangle1,reactangle2 =self.obstacles 
        if circle1.is_inside(x,y):
            collision = True
            print("Point inside the obstacle")
        elif square.is_inside(x,y):
            collision = True
            print("Point inside the obstacle")
        elif circle2.is_inside(x,y):
            collision = True
            print("Point inside the obstacle")

        elif reactangle1.is_inside(x,y):
            collision = True
            print("Point inside the obstacle")
        elif reactangle2.is_inside(x,y):
            collision = True
            print("Point inside the obstacle")

        else:
            collision = False

        return collision


class Circle():

    def __init__(self, x: int, y: int, radius: int, clearance : int) -> None:
        self.name = "Circle"
        self.x = x
        self.y = y
        self.radius = radius
        self.clearance = clearance


    def is_inside(self, x,y):
        if (x - self.x) **2 + (y- self.y)**2  < (self.radius+self.clearance)**2:

            return  True
        else:
            return False

    def add_plot(self,ax):
        outer_radius = self.clearance + self.radius
        clearance_circle = patches.Circle((self.x,self.y), radius=outer_radius, linewidth=0.5, alpha=0.5, edgecolor="black", facecolor="black")
        circle = patches.Circle((self.x,self.y), radius=self.radius, linewidth=0.5, alpha=0.5, edgecolor="black", facecolor="black")
        ax.add_patch(circle)
        ax.add_patch(clearance_circle)
        

class Rectangle:
    def __init__(self, x, y, w, h, clearance=0):
        self.type = 'rectangle'
        self.x, self.y = x,y
        self.w, self.h = w,h
        self.clearance  = clearance
        self.xlim = (x-w/2, x+w/2)
        self.ylim = (y- h/2, y+h/2)

    def is_inside(self, x,y):
        if  (self.xlim[0]-self.clearance  < x < self.xlim[1]+self.clearance ) and (self.ylim[0]-self.clearance  < y < self.ylim[1]+self.clearance )  :  
            return True
        else:
            return False

    def add_plot(self, ax):
        
        outer = (self.xlim[0]-self.clearance, self.ylim[0]-self.clearance)
        inner = (self.xlim[0], self.ylim[0])
        outer_rectangle = patches.Rectangle(outer, self.w+2*self.clearance, self.h+2*self.clearance, linewidth=0.5, alpha=0.5, edgecolor="gray", facecolor="gray")
        inner_rectangle = patches.Rectangle(inner, self.w, self.h, edgecolor="black", facecolor="black")
        ax.add_patch(outer_rectangle)
        ax.add_patch(inner_rectangle)



def main():
    fig, ax = plt.subplots()
    ax.set(xlim=(0, 10), ylim = (0,10))
    m = Map(10,10,0.1)
    ax = m.generate_map(ax)
    ax.set_aspect("equal")
    m.check_collision(1.1,1.4)
    plt.show()
if __name__ == '__main__':
    main()
