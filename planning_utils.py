from enum import Enum
from queue import PriorityQueue
import numpy as np


# \todo(michael): Modify this so that the cells of the grid contain
#                 the height information rather than a boolean
#                 indicating occupied or not. This would make it
#                 much easier to generate a boolean grid at any
#                 height by doing something like: 'grid > 5'
#  We can use np.maximum(a1, a2) to compare two arrays.
#  The procedure would be to get the chunk of the 'grid' array
#  and compare that to the new obstacle height.
def create_2p5d_map(data, safety_distance):
    """
    Returns a 2.5D grid representation of a configuration space
    based on given obstacle data and safety distance arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        height =  alt + d_alt + safety_distance
        obstacle = [
            int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
            int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
            int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
            int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
        ]
        tmp = np.ones((obstacle[1]-obstacle[0]+1, obstacle[3]-obstacle[2]+1)) * height
        nslice = slice(obstacle[0], obstacle[1]+1)
        eslice = slice(obstacle[2], obstacle[3]+1)
        grid[nslice, eslice] = np.maximum(tmp, grid[nslice, eslice])

    return grid, (int(north_min), int(east_min))

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """
    map_2p5d, ne_min = create_2p5d_map(data, safety_distance)

    return map_2p5d > drone_altitude, ne_min


# Assume all actions cost the same.
class Action2D(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    UP = (-1, 0, 1)
    URIGHT = (-1, 1, np.math.sqrt(2))
    RIGHT = (0, 1, 1)
    DRIGHT = (1, 1, np.math.sqrt(2))
    DOWN = (1, 0, 1)
    DLEFT = (1, -1, np.math.sqrt(2))
    LEFT = (0, -1, 1)
    ULEFT = (-1, -1, np.math.sqrt(2))


    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = [a for a in Action2D]
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle
    for a in Action2D:
        new_x = x + a.delta[0]
        new_y = y + a.delta[1]
        # Check if the node is off the grid
        if new_x < 0 or new_x > n or new_y < 0 or new_y > m:
            valid.remove(a)
        # Check if the node is an obstacle
        elif grid[new_x, new_y] == 1:
            valid.remove(a)

    return valid

def grid_get_children(grid, current_node):
    children = []
    for action in valid_actions(grid, current_node):
        da = action.delta
        children.append(((current_node[0] + da[0], current_node[1] + da[1]),
                          action.cost))
    return children


class ActionNED(Enum):
    """
    An action is represented by a 4 element tuple.

    The first 3 values are the delta of the action relative
    to the current grid position. The fourth and final value
    is the cost of performing the action.
    """

    # Movements in the NE plane
    NORTH = (1, 0, 0, 1)
    NORTHEAST = (1, 1, 0, np.sqrt(2))
    EAST = (0, 1, 0, 1)
    SOUTHEAST = (-1, 1, 0, np.sqrt(2))
    SOUTH = (-1, 0, 0, 1)
    SOUTHWEST = (-1, -1, 0, np.sqrt(2))
    WEST = (0, -1, 0, 1)
    NORTHWEST = (1, -1, 0, np.sqrt(2))
    # Movements downward
    DN = (0, 0, 1, 1)
    DN_NORTH = (1, 0, 1, np.sqrt(2))
    DN_NORTHEAST = (1, 1, 1, np.sqrt(3))
    DN_EAST = (0, 1, 1, np.sqrt(2))
    DN_SOUTHEAST = (-1, 1, 1, np.sqrt(3))
    DN_SOUTH = (-1, 0, 1, np.sqrt(2))
    DN_SOUTHWEST = (-1, -1, 1, np.sqrt(3))
    DN_WEST = (0, -1, 1, np.sqrt(2))
    DN_NORTHWEST = (1, -1, 1, np.sqrt(3))
    # Movements upward
    UP = (0, 0, -1, 1)
    UP_NORTH = (1, 0, -1, np.sqrt(2))
    UP_NORTHEAST = (1, 1, -1, np.sqrt(3))
    UP_EAST = (0, 1, -1, np.sqrt(2))
    UP_SOUTHEAST = (-1, 1, -1, np.sqrt(3))
    UP_SOUTH = (-1, 0, -1, np.sqrt(2))
    UP_SOUTHWEST = (-1, -1, -1, np.sqrt(3))
    UP_WEST = (0, -1, -1, np.sqrt(2))
    UP_NORTHWEST = (1, -1, -1, np.sqrt(3))

    @property
    def cost(self):
        return self.value[3]

    @property
    def delta(self):
        return (self.value[0], self.value[1], self.value[2])


def grid_3d_get_children(map_2p5d, current_node):
    valid = [a for a in ActionNED]
    n, m = map_2p5d.shape[0] - 1, map_2p5d.shape[1] - 1
    x, y, z = current_node

    children = []
    for a in ActionNED:
        new_x = x + a.delta[0]
        new_y = y + a.delta[1]
        new_z = z + a.delta[2]

        # Check if the node is off the grid
        if new_x < 0 or new_x > n or new_y < 0 or new_y > m or new_z < 0:
            continue
        if map_2p5d[new_x, new_y] > new_z:
            continue

        # If we've gotten here then the new point is valid
        children.append(((new_x, new_y, new_z), a.cost))

    return children


# Helper function for graph based search algorithm.
def graph_get_children(graph, current_node):
    children = []
    for next_node in graph.neighbors(current_node):
        cost = graph.get_edge_data(current_node, next_node)['weight']
        children.append((next_node, cost))
    return children

def a_star(get_children, h, start, goal):
    """
    A* planner, that has the following interface:
        'get_children' is a function that takes a node and returns
                       all valid child nodes.
        'h' is the heuristic function. It takes a node & goal position
            and returns the estimated cost to the goal.
        'start' is the starting node, a tuple.
        'goal'  is the terminal node, a tuple.
    """
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for child_node, cost in get_children(current_node):
                branch_cost = current_cost + cost
                queue_cost = branch_cost + h(child_node, goal)

                if child_node not in visited:
                    visited.add(child_node)
                    branch[child_node] = (branch_cost, current_node)
                    queue.put((queue_cost, child_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost


def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

