import time
import chaospy
import numpy as np
import numpy.linalg as LA
from sklearn.neighbors import KDTree
from shapely.geometry import LineString

import pkg_resources
pkg_resources.require("networkx>=2.1")
import networkx as nx

from planning_utils import a_star, heuristic, graph_get_children
from obstacle_lib import PolyLibrary, CollidersData


class ProbabilisticRoadMap:
    def __init__(self, data, num_samples=1000, zmin=0, zmax=-1):

        self._cdata = CollidersData(data)
        # Keep track of our own zmin & zmax in case we want to
        # narrow the sample space in the z-direction
        self._zmin = zmin
        if zmax < zmin:
            # Assume we should use the colliders max
            zmax = self._cdata.zrange - self._cdata.zmin
        self._zmax = zmax

        # Create a PolyLibrary object for collision checking.
        self._plib = PolyLibrary(data)

        nodes = self._sample_space(num_samples)

        t0 = time.time()
        self._graph = self._create_graph(nodes, max_conns=3, min_dist=5)
        time_taken = time.time() - t0
        print("Graph creation took {0} seconds ...".format(time_taken))
        print("Graph has {0} edges.".format(len(self._graph.edges)))

    def plan_path(self, start, goal):

        # Find the nearest node on the graph to the start position & add it
        # to the graph:
        near_start = self._find_nearest_point(start)
        print('Adding starting edge to graph: {} <-> {}'.format(start, near_start))
        self._graph.add_edge(start, near_start,
                             weight=LA.norm(np.array(start) - np.array(near_start)))

        # Do the same for the goal position:
        near_goal = self._find_nearest_point(goal)
        print('Adding goal edge to graph: {} <-> {}'.format(near_goal, goal))
        self._graph.add_edge(near_goal, goal,
                             weight=LA.norm(np.array(near_goal) - np.array(goal)))

        path, _ = a_star(lambda node: graph_get_children(self._graph, node),
                         heuristic, start, goal)
        print(len(path), path)
        return path

    def _sample_space(self, num_samples=1000):
        # Use a quasi-random sampling technique which has been shown to
        # produce slightly better coverage of the space
        r_vals = chaospy.create_halton_samples(num_samples, 3)

        # Points are sampled in the half-open set [0, 1) so we need to
        # perform an affine transform in order to get them into the
        # same coordinates as our grid.
        xrange, yrange = self._cdata.xrange, self._cdata.yrange
        zrange = self._zmax - self._zmin
        xmin, ymin, zmin = self._cdata.xmin, self._cdata.ymin, self._zmin
        samples = (np.diag([xrange, yrange, zrange]).dot(r_vals) +
                   np.array([[xmin, ymin, zmin]]).transpose()).transpose().tolist()

        t0 = time.time()
        nodes = []
        for point in samples:
            # Find the nearest polygons:
            h = self._plib.get_height_at_point(point)
            if h:
                nodes.append((point[0], point[1], h+zmin))
            else:
                nodes.append(point)
        time_taken = time.time() - t0
        print("Collision checking took {0} seconds ...".format(time_taken))
        print("Kept {0} of {1} points".format(len(nodes), len(samples)))
        print("Average node coverage area: {0}".format((xrange*yrange) / len(nodes)))
        print("Grid center: ({0}, {1})".format(0.5*(xrange + 2*xmin), 0.5*(yrange + 2*ymin)))
        print("Sample point center: ({0})".format(np.average(np.array(nodes), axis=0)))
        search_radius = ((3*xrange*yrange*zrange) / (4*np.pi))**(1. / 3)
        print("Recommended search radius: {0}".format(search_radius))

        return nodes

    def _create_graph(self, nodes, max_conns=5, min_dist=5):
        g = nx.Graph()

        p_tree = KDTree(nodes)

        for n1 in nodes:
            # Here we'll look for the k nearest nodes, but only connect
            # up to 'max_conns' of them.
            dist, node_idx = p_tree.query([n1], k=20)
            conns = 0
            for idx, d in zip(node_idx[0], dist[0]):
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
                    g.add_edge(tuple(n1), tuple(n2),
                               weight=LA.norm(np.array(n1) - np.array(n2)))
                if conns >= max_conns:
                    break

        return g

    def _can_connect(self, p1, p2):
        line = LineString([p1, p2])
        for poly, height in self._plib.polys_between(p1, p2):
            if line.intersects(poly) and min(p1[2], p2[2]) <= height:
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
        nearest_node = nodes[dist.argmin(), :]

        return tuple(nearest_node.tolist())
