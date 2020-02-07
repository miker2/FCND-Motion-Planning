import pkg_resources
import sys

try:
    pkg_resources.require("networkx>=2.1")
    import networkx as nx
except:
    !{sys.executable} -m pip install  networkx>=2.1
    pkg_resources.require("networkx>=2.1")
    import networkx as nx
try:
    import chaospy
except:
    !{sys.executable} -m pip install chaospy
    import chaospy

import time
import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point, LineString
from shapely.geometry import box as Box
from queue import PriorityQueue
from sklearn.neighbors import KDTree
import pdb


class PolyLibrary:
    def __init__(self, data):
        self._poly_center = []
        self._poly_dict = dict()
        
        
        self._extract_polygons(data)
        
        self._poly_tree = KDTree(self._poly_center)

    @property
    def polygons(self):
        return list(self._poly_dict.values())
    
    def nearest_polys(self, point, num_polys=1):
        ''' This function returns the nearest N polygons to the 2D point '''
        idx = self._poly_tree.query([point], k=num_polys,
                                       return_distance=False)[0]
        polys = []
        for i in idx:
            polys.append(self.__getitem__(i))
            
        return polys
    
    def polys_between(self, p1, p2):
        ''' Given two points, find all of the polygons that could potentially
            be along the line between them.
            We'll do this by calculating the distance between the two points
            and then using that as a search radius for polygons centered 
            around the two points. This will provide more polygons than necessary
            but will be far fewer than checking all of the polygons for collision
            '''
        d = LA.norm(np.array(p2) - np.array(p1))
        idx = self._poly_tree.query_radius([p1[:2], p2[:2]], r=d)
        idx = set(list(idx[0]) + list(idx[1]))
        polys = []
        for i in idx:
            polys.append(self.__getitem__(i))
        
        return polys

    def collides(self, point):
        ''' Check if the point collides with any of the 
            polygons in the PolyLibrary
        '''
        # Determine whether the point collides
        # with any obstacles.
        polygons = self.nearest_polys(point[:2])
        for p, h in polygons:
            if p.contains(Point(point)) and point[2] <= h:
                return True
        return False

    def get_height_at_point(self, point):   
        # Returns the height of the polygon containing
        # the point, None otherwise.
        polygons = self.nearest_polys(point[:2])
        for p, h in polygons:
            if p.contains(Point(point)) and point[2] <= h:
                return h
        return None

    def __getitem__(self, idx):
        return self._poly_dict[self._poly_center[idx]]
    
    def _extract_polygons(self, data):
        # This method should return a dictionary with key-value pairs of 
        # polygon_center : shapely polygon

        for i in range(data.shape[0]):
            north, east, alt, d_north, d_east, d_alt = data[i, :]

            # Extract the min & max extents of the obstacle and create a box shaped
            # polygon
            p = Box(north-d_north, east-d_east, 
                    north+d_north, east+d_east)        

            # Compute the height of the polygon
            height = alt+d_alt

            center = (north, east)
            self._poly_dict[center] = (p, height)
            self._poly_center.append(center)


            
def heuristic(n1, n2):
    return LA.norm(np.array(n1) - np.array(n2))

def a_star_graph(graph, h, start, goal):
    """Modified A* to work with NetworkX graphs."""
    
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
            for next_node in graph.neighbors(current_node):
                branch_cost = current_cost + graph.get_edge_data(current_node,next_node)['weight']
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))
             
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


class CollidersData:
    def __init__(self, data):
        self._data = data

        # Calculate the data necessary to do an affine transform from
        # the unit cube to this grid size.
        self._xmin = np.min(data[:, 0] - data[:, 3])
        xmax = np.max(data[:, 0] + data[:, 3])
        self._xrange =  xmax - self._xmin

        self._ymin = np.min(data[:, 1] - data[:, 4])
        ymax = np.max(data[:, 1] + data[:, 4])
        self._yrange = ymax - self._ymin

        self._zmin = np.min(data[:, 2] - data[:, 5])
        zmax = np.max(data[:, 2] + data[:, 5])
        self._zrange = zmax - self._zmin

    @property
    def xmin(self):
        return self._xmin

    @property
    def xrange(self):
        return self._xrange

    @property
    def ymin(self):
        return self._ymin

    @property
    def yrange(self):
        return self._yrange

    @property
    def zmin(self):
        return self._zmin

    @property
    def zrange(self):
        return self._zrange

    @property
    def data(self):
        return self._data
    
    def __getitem__(self, i, j):
        return self._data[i, j]


class ProbabalisticRoadMap:
    def __init__(self, data, num_samples=1000, zmin=0, zmax=-1):

        self._grid = CollidersData(data)
        # Keep track of our own zmin & zmax in case we want to
        # narrow the sample space in the z-direction
        self._zmin = zmin
        if zmax < zmin:
            # Assume we should use the colliders max
            zmax = self._grid.zrange - self._grid.zmin
        self._zmax = zmax

        # Create a PolyLibrary object for collision checking.
        self._PL = PolyLibrary(data)

        nodes = self._sample_space(num_samples)

        t0 = time.time()
        self._graph = create_graph(nodes, PL, max_conns=3, min_dist=5)
        time_taken = time.time() - t0
        print("Graph creation took {0} seconds ...".format(time_taken))
        print("Graph has {0} edges.".format(len(self._graph.edges)))

    def plan_path(self, start, goal):
        
        start = list(self._graph.nodes)[10]
        k = np.random.randint(len(self._graph.nodes))
        print(k, len(self._graph.nodes))
        goal = list(self._graph.nodes)[k]

        path, cost = a_star_graph(g, heuristic, start, goal)
        print(len(path), path)
        return path

    def _sample_space(self, num_samples):
        # Use a quasi-random sampling technique which has been shown to
        # produce slightly better coverage of the space
        r_vals = chaospy.create_halton_samples(1000, 3)

        # Points are sampled in the half-open set [0, 1) so we need to
        # perform an affine transform in order to get them into the
        # same coordinates as our grid.
        xrange, yrange, zrange = self._grid.xrange, self._grid.yrange, self._grid.zrange
        xmin, ymin, zmin = self._grid.xmin, self._grid.ymin, self._grid.zmin
        samples = (np.diag([xrange, yrange, zrange]).dot(r_vals) +
          np.array([[xmin, ymin, zmin]]).transpose()).transpose().tolist()

        t0 = time.time()
        nodes = []
        for point in samples:
            # Find the nearest polygons:
            h = self._PL.get_height_at_point(point)
            if h:
                nodes.append((point[0], point[1], h+zmin))
            else:
                nodes.append(point)
            #if not self._PL.collides(point):
            #    nodes.append(point)
        time_taken = time.time() - t0
        print("Collision checking took {0} seconds ...".format(time_taken))
        print("Kept {0} of {1} points".format(len(nodes), len(samples)))
        print("Average node coverage area: {0}".format((xrange*yrange)/len(nodes)))
        print("Grid center: ({0}, {1})".format(0.5*(xrange+2*xmin), 0.5*(yrange+2*ymin)))
        print("Sample point center: ({0})".format(np.average(np.array(nodes), axis=0)))
        search_radius = ((3*xrange*yrange*zrange)/(4*np.pi))**(1./3)
        print("Recommended search radius: {0}".format(search_radius))

        return nodes

    def _create_graph(self, nodes, max_conns=5, min_dist=5):
        g = nx.Graph()
    
        p_tree = KDTree(nodes)
    
        for n1 in nodes:
            # Here we'll look for the k nearest nodes, but only connect
            # up to 'max_conns' of them.
            dist, node_idx = p_tree.query([n1], k=15)
            conns = 0
            for idx, d in zip(node_idx[0],dist[0]):
                n2 = nodes[idx]
                if n1 == n2:
                    continue
                if d < min_dist:
                    print("skipping node {} due to min dist violation ({:.3} < {})".format(
                    n2, d, min_dist))
                    continue
                # Test the node for possible connectedness
                if can_connect(n1, n2, self._PL):
                    conns += 1
                    g.add_edge(n1, n2, weight=LA.norm(np.array(n1)-np.array(n2)))
                if conns >= max_conns:
                    break
                #if conns < max_conns:
                #    print("For node", n1, "only found {0} connections.".format(conns))
        return g




    
#=============================================================
# Rework this into a function (or just have data be passed in)

# filename = 'colliders.csv'
# data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)

# # Determine the extents of the map:
# xmin = np.min(data[:, 0] - data[:, 3])
# xmax = np.max(data[:, 0] + data[:, 3])
# xrange = xmax - xmin

# ymin = np.min(data[:, 1] - data[:, 4])
# ymax = np.max(data[:, 1] + data[:, 4])
# yrange = ymax - ymin

# zmin = 3
# # Limit the z axis for the visualization
# zmax = 50 # alt. np.max(data[:,2] + data[:,5])
# zrange = zmax-zmin

# stat_str = "min = {0}, max = {1}, range = {2}\n"

# print("X")
# print(stat_str.format(xmin, xmax, xrange))

# print("Y")
# print(stat_str.format(ymin, ymax, yrange))

# print("Z")
# print(stat_str.format(zmin, zmax, zrange))



# TODO: sample points randomly
# then use KDTree to find nearest neighbor polygon
# and test for collision

# Generate a dictionary of polygons, using the center as the key:
PL = PolyLibrary(data)

# Generate a uniform random sampling of points.
num_samples = 1000
np.random.seed(0)  # Seed random so we always get the same data.
                   # This can be changed later.

# Generate some random 3-dimensional points
#xvals = np.random.uniform(xmin, xmax, num_samples)
#yvals = np.random.uniform(ymin, ymax, num_samples)
#zvals = np.random.uniform(zmin, zmax, num_samples)

#r_vals = np.random.rand(num_samples, 3)
r_vals = chaospy.create_halton_samples(1000, 3).transpose()

xvals = r_vals[:,0]*xrange + xmin
yvals = r_vals[:,1]*yrange + ymin
zvals = r_vals[:,2]*zrange + zmin

samples = list(zip(xvals, yvals, zvals))

t0 = time.time()
nodes = []
for point in samples:
    # Find the nearest polygons:
    #print(point)
    #print(np.reshape(point[0:2],[1, -1]))
    polys = PL.nearest_polys(point[:2])
    h = get_point_height(polys, point)
    nodes.append((point[0], point[1], h))
    #if not collides(polys, point):
    #    nodes.append(point)
time_taken = time.time() - t0
print("Collision checking took {0} seconds ...".format(time_taken))
print("Kept {0} of {1} points".format(len(nodes), len(samples)))
print("Average node coverage area: {0}".format((xrange*yrange)/len(nodes)))
print("Grid center: ({0}, {1})".format(0.5*(xmax+xmin), 0.5*(ymax+ymin)))
print("Sample point center: ({0})".format(np.average(np.array(nodes), axis=0)))
search_radius = ((3*xrange*yrange*zrange)/(4*np.pi))**(1./3)
print("Recommended search radius: {0}".format(search_radius))

t0 = time.time()
g = create_graph(nodes, PL, max_conns=3, min_dist=5)
time_taken = time.time() - t0
print("Graph creation took {0} seconds ...".format(time_taken))
print("Graph has {0} edges.".format(len(g.edges)))


start = list(g.nodes)[10]
k = np.random.randint(len(g.nodes))
print(k, len(g.nodes))
goal = list(g.nodes)[k]

path, cost = a_star_graph(g, heuristic, start, goal)
print(len(path), path)
