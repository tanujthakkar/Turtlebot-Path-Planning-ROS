import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
HEIGHT, WIDTH = 10.0, 10.0 # [m], Height and Width of the world



class Map:

    def __init__(self, clearence):
        self.clearance = clearence
        self.height, self.width = HEIGHT, WIDTH    
        self.obstacles = self.createObstacles()

    def createObstacles(self):
        circle1 = Circle(2,2,1,self.clearance)
        square = Rectangle(1,5,1.5,1.5,self.clearance)
        circle2 = Circle(2,self.height -2,1,self.clearance)
        reactangle1 = Rectangle(5,5,2.5,1.5,self.clearance)
        reactangle2 = Rectangle(8,3,1.5,2,self.clearance) 

        self.obstacles = [circle1,square,circle2,reactangle1,reactangle2]
        return self.obstacles

    def check_collision(self,x,y):
        if not ((self.clearance <x< self.width-self.clearance) and  (self.clearance < y < self.height-self.clearance)):
            collision = True
            print("point outside map")
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

    def plot_all(self, ax):
        for obstacle in self.obstacles:
            obstacle.add_plot(ax)
        return ax 

    def is_inside_map(self, x, y):
        return (self.clearance <x< WIDTH-self.clearance and  self.clearance < y < HEIGHT-self.clearance)

    def inside_obstacle(self, x, y):
        state = False
        for obstacle in self.obstacles:
            state = state or obstacle.isInside(x,y)
        return state

class Circle:

    def __init__(self, x, y, radius, clearance=0):
        self.type = 'circle'
        self.x, self.y = x,y
        self.radius = radius
        self.clearance = clearance

    def is_inside(self, x,y):
        if (x - self.x) **2 + (y- self.y)**2  < (self.radius+self.clearance)**2:
            return  True
        else:
            return False

    def add_plot(self,ax):
        outer_radius = self.clearance + self.radius
        clearance_circle = patches.Circle((self.x,self.y), radius=outer_radius, linewidth=0.5, alpha=0.5, edgecolor="gray", facecolor="gray")
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
        if  (self.xlim[0]-self.clearance  < x < self.xlim[1]+self.clearance) and (self.ylim[0]-self.clearance  < y < self.ylim[1]+self.clearance ):
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
