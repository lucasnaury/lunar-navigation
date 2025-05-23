# Sources:
# - https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
# - https://gist.github.com/ryancollingwood/32446307e976a11a1185a5394d6657bc
# This is a modified version of the source code, with fuzzy logic

import cv2
import numpy as np
import heapq
from pathlib import Path
from warnings import warn
from include.helpers import showImage

class Node:
    """
    A node class for A* Pathfinding
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    
    def __repr__(self):
      return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    # defining less than for purposes of heap queue
    def __lt__(self, other):
      return self.f < other.f
    
    # defining greater than for purposes of heap queue
    def __gt__(self, other):
      return self.f > other.f


def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path


def astar(slopeMap, illuminationMap, start, end, weights, isDebug=False, gui=False):
    """
    Returns a list of tuples as a path from the given start to the given end in the given costmap
    :param costmap:
    :param start:
    :param end:
    :return:
    """

    # Create start and end node
    start_node = Node(None, start)
    end_node = Node(None, end)

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    if isDebug:
        cropped = [(0,slopeMap.shape[0]), (0,slopeMap.shape[1])]
        # cropped = [(4100, 5300), (4300, 6100)]
        size = (slopeMap.shape[0],slopeMap.shape[1],3)
        # size = (1200,1800,3)
        explored = np.zeros(size)
        explored[start] = (255,255,255)
        not_walkable = np.empty((0,2), int)


    # Heapify the open_list and Add the start node
    heapq.heapify(open_list) 
    heapq.heappush(open_list, start_node)

    # Adding a stop condition
    outer_iterations = 0
    max_iterations = len(slopeMap[0]) * len(slopeMap) * 5

    # what squares do we search
    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),)

    # Loop until you find the end
    while len(open_list) > 0:
        outer_iterations += 1

        if outer_iterations > max_iterations:
            # if we hit this point return the path such as it is
            # it will not contain the destination
            warn("giving up on pathfinding too many iterations")
            return return_path(current_node), explored  
        
        # Get the current node
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)


        # Debug
        if isDebug:
            print(f"{outer_iterations}/{max_iterations}: ", current_node.position, f"F{current_node.f:.02f} H{current_node.h}")
            
            # Show not walkable nodes
            if len(not_walkable) > 0:
                explored[not_walkable[:,0] - cropped[0][0], not_walkable[:,1]- cropped[1][0]] = (0,0,1.0)
            
            
            # Show current node
            explored_img = explored.copy()
            current_pos = (current_node.position[0] - cropped[0][0], current_node.position[1] - cropped[1][0])
            explored_img[current_pos] = (255,0,0)

            if gui:
                showImage("Explored", explored_img)
                cv2.waitKey(1)

        # Found the goal
        if current_node == end_node:
            if isDebug:
                return return_path(current_node), explored


            return return_path(current_node), None

        # Generate children
        children = []
        
        for new_position in adjacent_squares: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(slopeMap) - 1) or node_position[0] < 0 or node_position[1] > (len(slopeMap[len(slopeMap)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if slopeMap[node_position[0]][node_position[1]] >= 1:
                if isDebug:
                    not_walkable = np.vstack((not_walkable,node_position))
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            
            # Get costs
            distance_cost, slope_cost, illumination_cost, steering_cost = getCosts(child, slopeMap, illuminationMap)


            # Create the f, g, and h values with modified cost
            child.h = np.sqrt((child.position[0] - end_node.position[0]) ** 2 + (child.position[1] - end_node.position[1]) ** 2)
            # child.h = np.abs(child.position[0] - end_node.position[0]) + np.abs(child.position[1] - end_node.position[1])

            w_dist = weights[0]
            w_steering = weights[1]
            w_slope = weights[2]
            w_light = weights[3]

            w_g = weights[4]
            w_h = weights[5]

            child.g = current_node.g + (w_dist*distance_cost + w_steering*steering_cost + w_slope*slope_cost + w_light*illumination_cost) / (w_dist+w_steering+w_slope+w_light)
            child.f = (w_g*child.g + w_h*child.h) / (w_g + w_h)

            if isDebug:
                child_pos = (child.position[0] - cropped[0][0], child.position[1] - cropped[1][0])
                explored[child_pos] = (255,255,255)

            # Child is already in the open list
            if len([open_node for open_node in open_list if child.position == open_node.position and child.g > open_node.g]) > 0:
                continue

            # Add the child to the open list
            heapq.heappush(open_list, child)

    warn("Couldn't get a path to destination")
    return None, explored


# IMPORTANT PART
def getCosts(child:Node, slopeMap, illuminationMap):

    # -> Slope cost
    slope_cost = slopeMap[int(child.position[0])][int(child.position[1])]

    # -> Illumination cost (opposite of illumination map)
    illumination_cost = 1.0 - illuminationMap[int(child.position[0])][int(child.position[1])]
    
    
    # -> Distance cost
    vect = [child.position[0] - child.parent.position[0],
            child.position[1] - child.parent.position[1]]
    distance_cost = np.sqrt(vect[0]**2 + vect[1]**2)


    if child.parent.parent is None:
        # return (distance_w*distance_cost + costmap_w*costmap_cost) / (distance_w+costmap_w)
        # return distance_cost + weight*costmap_cost
        return distance_cost, slope_cost, illumination_cost, 0



    # -> Steering cost
    new_heading = np.arctan2(vect[0], vect[1])

    vect = [child.parent.position[0] - child.parent.parent.position[0],
            child.parent.position[1] - child.parent.parent.position[1]]
    current_heading = np.arctan2(vect[0], vect[1])

    steering_cost = np.abs(new_heading - current_heading) / np.pi

    # return (distance_w * distance_cost + costmap_w * costmap_cost + steering_w * steering_cost) / (distance_w + costmap_w + steering_w)
    # return distance_cost + weight*(costmap_w * costmap_cost + steering_w * steering_cost) / (costmap_w + steering_w)
    
    return distance_cost, slope_cost, illumination_cost, steering_cost


def test():

    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    start = (0, 0)
    end = (7, 6)

    path = astar(maze, start, end)
    print(path)


if __name__ == '__main__':
    test()