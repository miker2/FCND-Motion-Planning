import numpy as np
import numpy.linalg as LA
from sklearn.neighbors import KDTree
from shapely.geometry import Point
from shapely.geometry import box as Box


class PolyLibrary:
    """
    PolyLibrary is an interface class to the underlying collision regions of the map.
    It provides an easy interface for finding the height of an object at a point,
    The polygons between any two points, and collision checking of a specified point.
    """
    def __init__(self, data):
        self._poly_center = []
        self._poly_dict = dict()

        self._extract_polygons(data)

        self._poly_tree = KDTree(self._poly_center)

    @property
    def polygons(self):
        """ Provides the list of shapely polygon objects in the library """
        return list(self._poly_dict.values())

    def nearest_polys(self, point, num_polys=1):
        """ This function returns the nearest N polygons to the 2D point """
        idx = self._poly_tree.query([point], k=num_polys,
                                    return_distance=False)[0]
        polys = []
        for i in idx:
            polys.append(self.__getitem__(i))

        return polys

    def polys_between(self, p1, p2):
        """
        Given two points, find all of the polygons that could potentially
        be along the line between them.
        We'll do this by calculating the distance between the two points
        and then using that as a search radius for polygons centered
        around the two points. This will provide more polygons than necessary
        but will be far fewer than checking all of the polygons for collision
        """
        d = LA.norm(np.array(p2) - np.array(p1))
        idx = self._poly_tree.query_radius([p1[:2], p2[:2]], r=d)
        idx = set(list(idx[0]) + list(idx[1]))
        polys = []
        for i in idx:
            polys.append(self.__getitem__(i))

        return polys

    def collides(self, point):
        """
        Check if the point collides with any of the polygons in the PolyLibrary
        """
        # Determine whether the point collides
        # with any obstacles.
        polygons = self.nearest_polys(point[:2])
        for p, h in polygons:
            if p.contains(Point(point)) and point[2] <= h:
                return True
        return False

    def get_height_at_point(self, point):
        """
        This function returns the height of the polygon containing
        the point, None otherwise.
        """
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
            p = Box(north - d_north, east - d_east,
                    north + d_north, east + d_east)

            # Compute the height of the polygon
            height = alt + d_alt

            center = (north, east)
            self._poly_dict[center] = (p, height)
            self._poly_center.append(center)


class CollidersData:
    """
    CollidersData is a thin wrapper around the data that comes out of the colliders.csv
    file. It provides information about the dimensions of the underlying map.
    """
    def __init__(self, data):
        self._data = data

        # Calculate the data necessary to do an affine transform from
        # the unit cube to this grid size.
        self._xmin = np.min(data[:, 0] - data[:, 3])
        xmax = np.max(data[:, 0] + data[:, 3])
        self._xrange = xmax - self._xmin

        self._ymin = np.min(data[:, 1] - data[:, 4])
        ymax = np.max(data[:, 1] + data[:, 4])
        self._yrange = ymax - self._ymin

        self._zmin = np.min(data[:, 2] - data[:, 5])
        zmax = np.max(data[:, 2] + data[:, 5])
        self._zrange = zmax - self._zmin

    @property
    def xmin(self):
        """ Returns the minimum x coordinate of any obstacle in colliders map """
        return self._xmin

    @property
    def xrange(self):
        """ Returns the range of obstacle along x-axis in colliders map """
        return self._xrange

    @property
    def ymin(self):
        """ Returns the minimum y coordinate of any obstacle in colliders map """
        return self._ymin

    @property
    def yrange(self):
        """ Returns the range of obstacle along y-axis in colliders map """
        return self._yrange

    @property
    def zmin(self):
        """ Returns the minimum z coordinate of any obstacle in colliders map """
        return self._zmin

    @property
    def zrange(self):
        """ Returns the range of obstacle along z-axis in colliders map """
        return self._zrange

    @property
    def min(self):
        """ Returns an (x, y, z) tuple of minimum coordinates. """
        return (self.xmin, self.ymin, self.zmin)

    @property
    def bounds2D(self):
        """ Provides the (xmin, ymin, xmax, ymax) bounds of the colliders map. """
        return (self.xmin, self.ymin, self.xmin + self.xrange, self.ymin + self.yrange)

    @property
    def data(self):
        """ Direct access to underlying data array """
        return self._data

    def __getitem__(self, idx):
        """ Access the rows of the underlying data """
        return self._data[idx, :]
