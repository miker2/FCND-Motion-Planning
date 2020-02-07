import pkg_resources
import sys

pkg_resources.require("networkx>=2.1")
import networkx as nx
import chaospy

import time
import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point, LineString
from shapely.geometry import box as Box
from queue import PriorityQueue
from sklearn.neighbors import KDTree
from planning_utils import a_star, heuristic

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


# Helper function for ProbabilisticRoadMap graph search algorithm.
def _graph_get_children(graph, current_node):
    children = []
    for next_node in graph.neighbors(current_node):
        cost = graph.get_edge_data(current_node, next_node)['weight']
        children.append((next_node, cost))
    return children

class ProbabilisticRoadMap:
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
        self._graph = self._create_graph(nodes, max_conns=3, min_dist=5)
        time_taken = time.time() - t0
        print("Graph creation took {0} seconds ...".format(time_taken))
        print("Graph has {0} edges.".format(len(self._graph.edges)))

    def plan_path(self, start, goal):

        #start = list(self._graph.nodes)[10]
        #k = np.random.randint(len(self._graph.nodes))
        #print(k, len(self._graph.nodes))
        #goal = list(self._graph.nodes)[k]

        # Find the nearest node on the graph to the start position & add it
        # to the graph:
        near_start = self._find_nearest_point(start)
        self._graph.add_edge(start, near_start,
                             weight=LA.norm(np.array(start)-np.array(near_start)))

        # Do the same for the goal position:
        near_goal = self._find_nearest_point(goal)
        self._graph.add_edge(goal, near_goal,
                             weight=LA.norm(np.array(goal)-np.array(near_goal)))

        path, cost = a_star(lambda node: _graph_get_children(self._graph, node),
                            heuristic, start, goal)
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
                if self._can_connect(n1, n2):
                    conns += 1
                    #print("n1: {}, n2: {}".format(n1, n2))
                    g.add_edge(tuple(n1), tuple(n2),
                               weight=LA.norm(np.array(n1)-np.array(n2)))
                if conns >= max_conns:
                    break
                #if conns < max_conns:
                #    print("For node", n1, "only found {0} connections.".format(conns))
        return g

    def _can_connect(self, p1, p2):
        l = LineString([p1, p2])
        for poly, height in self._PL.polys_between(p1, p2):
            if l.crosses(poly) and min(p1[2], p2[2]) <= height:
                return False

        return True

    def _find_nearest_point(self, point):
        ## Given a point (presumably the start or goal point, we want
        ## to find the nearest collision free point on the graph. This
        ## may be a node on the graph or it may be along an edge of the
        ## graph.

        ## If we wanted to find the nearest location on the graph, where
        #  the graph is composed of nodes connected by edges in cartesian
        #  space, then we might be able to do the following:
        #  (1) Find the nearest node
        #  (2) Find all neighbors of that node
        #  (3) Construct the lines between the nearest node and each of
        #      its neighbors
        #  (4) Project the point onto each of the lines and find the
        #      minimum distance point among the projections.


        # If we just want to look only for the nearest node, we can do
        # the following:
        nodes = np.array(self._graph.nodes)
        # Calculate the distance to all nodes:
        dist = np.linalg.norm(nodes - np.array(point), axis=1)
        # Find the index of the minimum distance and get the
        # corresponding node
        nearest_node = nodes[dist.argmin(),:]

        return tuple(nearest_node.tolist())


