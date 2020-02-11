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


class VoronoiPlanner:

    def __init__(self, data, safety_distance):
        
        self._PL = PolyLibrary(data)
        self._safety_distance = safety_distance

        self._CD = CollidersData(data)

    # A bit faster, but misses a few edges in narrow corridors
    def create_voronoi_grid(self, drone_altitude):
        """
        Returns a grid representation of a 2D configuration space
        along with Voronoi graph edges given obstacle data and the
        drone's altitude.
        """

        grid, centers = self._get_grid_and_centers(drone_altitude)

        # Convert centers to grid coordinates:
        grid_min = np.array(self._CD.min)[0:2]
        centers = (np.array(centers) - grid_min).tolist()
                    
        # Create a voronoi graph based on location of obstacle centres
        graph = Voronoi(centers)
        # voronoi_plot_2d(graph)  # uncomment to view raw voronoi graph

        north_size, east_size = grid.shape
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
               grid[p1i[0], p1i[1]] or grid[p2i[0], p2i[1]]:
                continue
                
            # Then you can test each pair p1 and p2 for collision using Bresenham
            # (need to convert to integer if using prebuilt Python package)
            # If the edge does not hit an obstacle add it to the list
            line = bresenham(p1i[0], p1i[1], p2i[0], p2i[1])
            #print([p for p in pts])
            # Check to see if any of the points are in collision
            in_collision = np.any([grid[p[0], p[1]] for p in line])
            if not in_collision:
                edges.append((tuple(p1 + grid_min), tuple(p2 + grid_min)))
    
        return self._create_graph(edges)


    def create_voronoi(self, drone_altitude):
        """
        Returns a Voronoi graph edges given obstacle data and the
        drone's altitude.
        """
        grid, centers = self._get_grid_and_centers(drone_altitude)

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

        return g

