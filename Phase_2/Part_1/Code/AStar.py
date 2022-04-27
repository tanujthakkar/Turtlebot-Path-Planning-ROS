
try:
   import queue 
except ImportError:
   import Queue as queue
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import argparse, time, csv, math
from utils import *
from MapGenerator import Map

class Node:

    def __init__(self, state, parent, move, cost, path_array): 
        self.state = state
        self.parent = parent
        self.move = move
        self.cost = cost
        self.path_Array = path_array


class Astar(Map):

    def __init__(self,  start_state, goal_state, clearance, rpm1, rpm2, save_path):
        self.radius, self.wheelDistance, self.dt = 0.038, 0.354, 0.1 
        Map.__init__(self, clearance)
        self.save_path = foldercheck(save_path)

        self.start, self.goal = start_state, goal_state
        self.RPM1 = rpm1
        self.RPM2 = rpm2
        self.actions = np.array([[0, self.RPM1],
                                 [self.RPM1, 0],
                                 [self.RPM1, self.RPM1],
                                 [0, self.RPM2],
                                 [self.RPM2, 0],
                                 [self.RPM2, self.RPM2],
                                 [self.RPM1, self.RPM2],
                                 [self.RPM2, self.RPM1]])

        self.visualize = False
        self.ax= None
        self.i=0
        self.final_moves, self.waypoints, self.final_points = [], [], []

    def reset(self):
        self.visualize=False
        self.ax= None
        self.i=0
        self.final_moves, self.waypoints, self.final_points = [], [], []

    def search(self):
        path = []
    
        print("Computing path")
        init_node = Node(self.start, None, None, 0, None)
        heap =  queue.PriorityQueue()
        heap.put((init_node.cost, init_node))
        threshold = 0.5
        if self.visualize:         
            self.ax.add_patch(patches.Circle( (self.start[0],self.start[1]), radius=0.1, linewidth=1, alpha=0.5, edgecolor='r', facecolor='r'))
            self.ax.add_patch(patches.Circle( (self.goal[0],self.goal[1]), radius=0.1, linewidth=1, alpha=0.5, edgecolor='g', facecolor='g'))
            plt.savefig(self.save_path+"/"+str(self.i)+".png")
            
            self.i+=1

        visibility_map = np.array([[[np.inf for k in range(360)] for j in range(int(self.height/threshold))] for _ in range(int(self.width/threshold))])
        while (not heap.empty()):
            currentNode = heap.get()[1]
            
            if self.euclidean(currentNode.state, thresh_radius=0.2):
                self.final_moves, node_path = self.backtrack(currentNode)                
                self.get_points(node_path)
                break
        
            for next_state in self.get_next_state(currentNode):            
                
                if self._isVisited(next_state, visibility_map,  threshold=0.5):

                    new_state = next_state.state
                    cost = self.compute_cost(new_state)
                    
                    nx = int(half_round(new_state[0])/threshold)
                    ny = int(half_round(new_state[1])/threshold)
                    visibility_map[nx, ny, new_state[2]] = next_state.cost + cost
                    
                    heap.put((next_state.cost + cost, next_state))

        return self.final_moves, self.waypoints, self.final_points

    
    def compute_cost(self, node_state, h=1.5):
        if node_state is None: return 0.0
        return h*((node_state[0]-self.goal[0])**2 + (node_state[1]-self.goal[1])**2)**(0.5)

    def _isVisited(self, node, visibility_map, threshold=0.5):
        x, y, theta = node.state
        x = int(half_round(x)/threshold)
        y = int(half_round(y)/threshold)
        total_cost = node.cost + self.compute_cost(node.state)
        return (total_cost < visibility_map[x, y, theta])

    def motion_model(self, state, action, T=1, path_mode=False):
        t = 0
        r, L, dt = self.radius, self.wheelDistance, self.dt
        
        Xn, Yn, theta_new = state
        theta_new = deg2rad(theta_new)
        
        uL, uR = action
        # Xn, Yn, theta_new = Xi, Yi, thetai
        path_array = [[Xn, Yn]]
        cost = 0.0

        while t<T:
            t = t + dt
            Xo, Yo = Xn, Yn
            dx = 0.5 * r * (uL + uR) * math.cos(theta_new) * dt
            dy = 0.5 * r * (uL + uR) * math.sin(theta_new) * dt
            Xn += dx
            Yn += dy
            theta_new += (r / L) * (uL - uR) * dt
            cost += math.sqrt(math.pow(dx,2) + math.pow(dy,2))
            path_array.append([Xn, Yn])

            if(self.visualize):
                if path_mode:
                    self.ax.plot([Xo, Xn], [Yo, Yn], color='g', linewidth = 2)
                else:
                    self.ax.plot([Xo, Xn], [Yo, Yn], color='r', alpha=0.1)

        if(self.visualize):
            plt.savefig(self.save_path+"/"+str(self.i)+".png")
            self.i+=1
            time.sleep(0.05)

        theta_new = int(rad2deg(theta_new))
        return [Xn, Yn, theta_new] , path_array, cost

    def get_next_state(self, node):
        state = node.state
        states = []
        for action in self.actions:
            new_state, path_array, step_cost = self.motion_model(state, action)
            x,y, _ = new_state
            if not (self.check_collision(x,y)):    
        
                new_node = Node(new_state, node, action, node.cost + step_cost, path_array)
                states.append(new_node)

        return states

    def get_points(self, node_path):
        for node in node_path:
            xi, yi, thetai = node.state
            self.waypoints.append([xi, yi, thetai])

            points = node.path_Array
            if points is not None:
                for point in points:
                    xn, yn = point
                    self.final_points.append([xn, yn])
                    xi, yi = xn, yn
    

    def euclidean(self, current_state, thresh_radius):
        return  np.square(current_state[0] - self.goal[0]) + np.square(current_state[1] - self.goal[1]) < thresh_radius**2

    def get_map(self):
        fig, self.ax = plt.subplots(figsize = (5, 5))
        self.ax.set(xlim=(0, 10), ylim = (0,10))
        self.ax = self.plot_all(self.ax) 
        self.ax.set_aspect("equal")
        
    def backtrack(self,current_node):
        moves, nodes = [], []

        while(current_node.move is not None):
            moves.append(current_node.move)
            nodes.append(current_node)
            current_node = current_node.parent
        nodes.append(current_node)
        
        moves.reverse()
        nodes.reverse()
        return moves, nodes
    
    def visualize_astar(self):
        self.get_map()
        self.visualize = True
        actions, waypoints, points  = self.search()
    
        assert len(waypoints) ==  len(actions)+1
        for idx in range(len(waypoints)-1):
            state = waypoints[idx]
            action = actions[idx] 
            self.motion_model(state, action, path_mode=True)
        return True

def main():
    Parser = argparse.ArgumentParser()
    Parser.add_argument('--start', default="1, 1, 30", help='Initial state of the turtlebot')
    Parser.add_argument('--goal', default="1, 2, 0", help='Goal state of the turtlebot')
    Parser.add_argument('--rpm', default="50, 100", help='Left and Right wheel RPMs')
    Parser.add_argument('--clearance', default="0.1", help='Clearance to obstacles')
    Parser.add_argument('--save_path', default="../Results/", help='Path to save results')

    Args = Parser.parse_args()
    start_state = np.array([int(e) for e in Args.start.split(',')])
    goal_state = np.array([int(e) for e in Args.goal.split(',')])
    rpm1, rpm2 = np.array([int(e) for e in Args.rpm.split(',')])
    clearance = float(Args.clearance)
    save_path = os.path.join(Args.save_path, Args.start_state)
   
    astar = Astar(start_state, goal_state, clearance,rpm1, rpm2, save_path)

    if(astar.visualize_astar()):
        generate_video(save_path, video_name= Args.start_state, fps=24)

if __name__ == "__main__":
    main()
