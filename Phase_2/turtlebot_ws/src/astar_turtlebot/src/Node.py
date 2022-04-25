import numpy as np

class Node():

    def __init__(self, state, parent, move, cost, path_array): 
        self.state = state
        self.parent = parent
        self.move = move
        self.cost = cost
        self.path_array = path_array
        
    def get_state(self):
        return self.state

    def get_path_array(self):
        return self.path_array
		
    def get_parent(self):
        return self.parent

    def get_parent_state(self):
        if self.get_parent() is None:
            return None
        return self.get_parent().get_state()
		
    def get_move(self):
	    return self.move
		
    def get_cost(self):
        return self.cost

    def get_path(self):
        moves = list()
        nodes = list()
        current_node = self
        
        while(current_node.get_move() is not None):
            moves.append(current_node.get_move())
            nodes.append(current_node)
            current_node = current_node.get_parent()

        nodes.append(current_node)
        moves.reverse()
        nodes.reverse()
        
        return moves, nodes