import pkg_resources
import time
from itertools import compress

import numpy as np
import numpy.linalg as LA
from scipy.spatial import Voronoi
from bresenham import bresenham

import shapely
import shapely.ops, shapely.prepared
from prm import PolyLibrary, CollidersData
from planning_utils import create_grid

pkg_resources.require("networkx>=2.1")
import networkx as nx

from planning_utils import a_star, heuristic, graph_get_children


class VoronoiPlanner:

    def __init__(self, data, safety_distance):
        
        self._PL = PolyLibrary(data)
        self._safety_distance = safety_distance

        self._CD = CollidersData(data)

        self._grid = None

        self._graph = None

    # A bit faster, but misses a few edges in narrow corridors
    def create_voronoi_grid(self, drone_altitude):
        """
        Returns a grid representation of a 2D configuration space
        along with Voronoi graph edges given obstacle data and the
        drone's altitude.
        """

        self._grid, centers = self._get_grid_and_centers(drone_altitude)

        # Convert centers to grid coordinates:
        grid_min = np.array(self._CD.min)[0:2]
        centers = (np.array(centers) - grid_min).tolist()
                    
        # Create a voronoi graph based on location of obstacle centres
        graph = Voronoi(centers)
        # voronoi_plot_2d(graph)  # uncomment to view raw voronoi graph

        north_size, east_size = self._grid.shape
        # Check each edge from graph.ridge_vertices for collision
        edges = []
        for v in graph.ridge_vertices:
            p1 = graph.vertices[v[0]]
            p2 = graph.vertices[v[1]]
            # Converting to int here would technically modify the 
            # voronoi diagram, so we'll use a new var name.
            p1i = [int(p) for p in p1]
            p2i = [int(p) for p in p2]
        
            if p1[0] < 0 or p1[0] >= north_size or \
               p2[0] < 0 or p2[0] >= north_size or \
               p1[1] < 0 or p1[1] >= east_size or \
               p2[1] < 0 or p2[1] >= east_size or \
               self._grid[p1i[0], p1i[1]] or self._grid[p2i[0], p2i[1]]:
                continue
                
            # Then you can test each pair p1 and p2 for collision using Bresenham
            # (need to convert to integer if using prebuilt Python package)
            # If the edge does not hit an obstacle add it to the list
            line = bresenham(p1i[0], p1i[1], p2i[0], p2i[1])
            #print([p for p in pts])
            # Check to see if any of the points are in collision
            in_collision = np.any([self._grid[p[0], p[1]] for p in line])
            if not in_collision:
                edges.append((tuple(p1 + grid_min), tuple(p2 + grid_min)))
    
        return self._create_graph(edges)


    def create_voronoi(self, drone_altitude):
        """
        Returns a Voronoi graph edges given obstacle data and the
        drone's altitude.
        """
        self._grid, centers = self._get_grid_and_centers(drone_altitude)

        polys = self._build_collision_regions(drone_altitude)
        polys = shapely.prepared.prep(polys)
        # Create a voronoi graph based on location of obstacle centres
        graph = Voronoi(centers)
        #voronoi_plot_2d(graph)  # uncomment to view raw voronoi graph
    
        x_min, y_min, x_max, y_max = self._CD.bounds2D
    
        # Check each edge from graph.ridge_vertices for collision
        edges = []
        for v in graph.ridge_vertices:
            p1 = graph.vertices[v[0]]
            p2 = graph.vertices[v[1]]
        
            if p1[0] < x_min or p1[0] > x_max or \
               p2[0] < x_min or p2[0] > x_max or \
               p1[1] < y_min or p1[1] > y_max or \
               p2[1] < y_min or p2[1] > y_max or \
               polys.intersects(shapely.geometry.LineString([tuple(p1), tuple(p2)])):
                continue
                
            edges.append((tuple(p1), tuple(p2)))
    
        return self._create_graph(edges)


    def plan_path(self, start, goal):

        #start = list(self._graph.nodes)[10]
        #k = np.random.randint(len(self._graph.nodes))
        #print(k, len(self._graph.nodes))
        #goal = list(self._graph.nodes)[k]

        if not self._graph:
            h = max(self._PL.get_height_at_point(start),
                    self._PL.get_height_at_point(goal))
            self._create_voronoi(h)

        start = (start[0], start[1])
        goal = (goal[0], goal[1])
        # Find the nearest node on the graph to the start position & add it
        # to the graph:
        near_start = self._find_nearest_point(start)
        print('Adding starting edge to graph: {} <-> {}'.format(start, near_start))
        self._graph.add_edge(start, near_start,
                             weight=LA.norm(np.array(start)-np.array(near_start)))

        # Do the same for the goal position:
        near_goal = self._find_nearest_point(goal)
        print('Adding goal edge to graph: {} <-> {}'.format(near_goal, goal))
        self._graph.add_edge(near_goal, goal,
                             weight=LA.norm(np.array(near_goal)-np.array(goal)))

        path, cost = a_star(lambda node: graph_get_children(self._graph, node),
                            heuristic, start, goal)
        print(len(path), path)
        return path

    
    def _get_grid_and_centers(self, drone_altitude):
        grid, grid_min = create_grid(self._CD.data, drone_altitude, self._safety_distance)
    
        centers = self._CD.data[:,0:2].tolist()
        height = self._CD.data[:,2] + self._CD.data[:,5] + self._safety_distance
        centers = list(compress(centers, (height > drone_altitude).tolist()))

        return grid, centers


    def _build_collision_regions(self, drone_altitude):

        t0 = time.time()
        polys_buffered = shapely.ops.unary_union(
            [p.buffer(self._safety_distance,
                      resolution=1) for p, h in self._PL.polygons \
             if h > drone_altitude - self._safety_distance])
        boundary = shapely.geometry.box(*self._CD.bounds2D)
        polys_buffered = boundary.intersection(polys_buffered)
        e_time = time.time() - t0
        print("Reduced {} polygons to {} polygons in {} seconds".format(
            len(self._PL.polygons), len(polys_buffered.geoms), e_time))

        return polys_buffered


    def _create_graph(self, edges):
        g = nx.Graph()

        for edge in edges:
            g.add_edge(edge[0], edge[1],
                       weight=LA.norm(np.array(edge[0]) - np.array(edge[1])))

        self._graph = g
        
        return g

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
