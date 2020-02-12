## Project: 3D Motion Planning

---

# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

---
## Writeup

### The Starter Code

#### 1. `motion_planning.py`

This script is very similar to the backyard flyer script. It uses event driven programming to command the quadrotor to arm, take-off, follow waypoints, land and then disarm.

The key differences between `motion_planning.py` and `backyard_flyer.py` is that
`backyard_flyer.py` follows a fixed set of waypoints in the shape of a box and
`motion_planning.py` has an added state as part of the finite state machine that
allows for planning a path through the urban environment. While the path that is
planned in this skeleton implementation is quite simple, all of the necessary
code is present to plan more complicated paths. One just needs to modify the body
of the `plan_path` method.

#### 2. `planning_utilities.py`

This script contains some of the key building blocks of a path planning algorithm
that we have been developing and using in the Planning unit of this nanodegree
program. The script includes the following functions/classes:

  1. `create_grid`  
    The method we have been using to generate a discretized grid based on the
obstacles described by `colliders.csv`.

  2. `Action`  
    The class that has been used by the A* algorithm to describe all possible
actions that will produce a new state from a current state. This class also provides
the cost associated with that action.

  3. `valid_actions`  
    This method prunes the list of all possible actions down to a list of valid actions
based on the configuration space (map extents and obstacle locations).

  4. `a_star`  
    Basic A* algorithm suited to planning a path through a grid based
2D configuration space.

  5. `heuristic`  
    Provides the estimated cost from a state to a goal position. The
heuristic used here is euclidian distance.

### Implementing The Path Planning Algorithm

#### 1. Set your global home position
I used the following code to read the latitude & longitude from the `colliders.csv` file.

    # Read lat0, lon0 from colliders into floating point values
    lat_lon_str = 'lat0 0, lon0 0'  # default in case file read fails.
    with open('colliders.csv') as f:
        lat_lon_str = f.readline()  # lat/lon is on the first line, only call `readline` once.
        f.close()                   # Close the file when done!

    # The lat/lon string sort of looks like a comma separate list of key/value
    # pairs, so we'll parse it under that assumption.
    lat_lon_dict = dict(item.split() for item in lat_lon_str.split(','))
		# Convert to float:
		for k, v in lat_lon_dict.items():
		    lat_lon_dict[k] = float(v)
    # set home position to (lon0, lat0, 0)
    self.set_home_position(lat_lon_dict['lon0'],
                           lat_lon_dict['lat0'],
                           0.0)

There are many ways to parse the string into the float values including regular expressions,
splitting the string on the command and then selecting everything from the 5th character to
the end of each of the two strings, etc, but I opted for a method that would be slightly
more robust in the event that the `lat0` and `lon0` entries in the file swapped places. While
this won't happen for this project, a dictionary is still a convenient way to accessing the
information by name.

#### 2. Set your current local position
We don't actually need to set the current local position here. While we can use
`self.global_position` and `self.global_home` (which we just updated by calling
`self.set_home_position(...)`) to calculate our current local position, we can also
access the `local_position` attribute of our class to get the local position, which
is what I do. In addition, `local_position` is a read-only attribute and is not settable.

#### 3. Set grid start position from local position
The local position is the quadrotor's current position relative to the global home. Since we
set the global home to the location of the center of the map/grid, then we want to make sure
that we set the grid start position to the `local_position` and then offset that by the grid
offset (`north_offset` and `east_offset`) This means that whenever the `plan_path` method
is called, the plan will always start from the quadrotor's current position.

In order to facilitate this I wrote a new method `local_to_grid` which takes a local position
and converts it into a position in the grid. It looks like this:

    def local_to_grid(self, local_position):
		    # Offset the local (NED) position by the grid offset, and cast to int.
        return (int(round(local_position[0]) - self._grid_offset[0]),
                int(round(local_position[1]) - self._grid_offset[1]))

In order to set the starting point of the planning problem I added the following line to
the `plan_path` method:

    grid_start = self.local_to_grid(self.local_position)

#### 4. Set grid goal position from geodetic coords
I achieved this by again creating another helper function `global_to_grid` which converts a
global geodetic position to a local position and then calls the `local_to_grid` function to
set the goal position. The helper method looks like this:

    def global_to_grid(self, global_position):
         _local_position = global_to_local(global_position, self.global_home)
         return self.local_to_grid(_local_position)

And the line of code I added is the following:

    # Convert the lat/lon goal location into a grid location:
    grid_goal = self.global_to_grid(goal_location)


#### 5. Modify A* to include diagonal motion (or replace A* altogether)
I investigated a variety of different planning methods in order to solve the planning
problem. I started by extending the grid-based A* planner to include diagonal motions
that have a cost of sqrt(2). This worked fine, but seemed to take a long time to find
a plan for goals far from the start location (as expected).

I knew it would be an even slower approach, but I experimented with a 3D voxel-based
A* planner by adding additional actions that allowed upward and downward motion as
well. This resulted in a total of 26 possible actions (connecting all adjacent voxels.
While the search space was expanded considerably leading to the possibility of flying
over buildings instead of just around them, the planning time was prohibitively long.
The good thing was that it was easy to expand from 2D to 3D grid based planning by
extending the `Action` class. I actually renamed the existing `Action` class to
`Action2D` and created a new class for the 3D version of the planner called `ActionNED`
for planning in the 3D North-East-Down frame.

Another approach I took to the planning problem was implementing a Probabilistic Road
Map planner. I did this by sampling the configuration space in three dimensions (only
accounting for translation motion and ignoring rotational motion). I then followed the
process of pruning nodes that weren't in free space, connecting nodes to create edges
in the graph, checking those edges for collision and building up the graph. While this
approach seemed quite fast when running in a test environment (see
`ProbabilisticRoadMap_test.ipynb`) the plannings took almost 100x longer to generate
when run from inside the simulator. I never could figure out what was causing this
slowdown.

Finally, I settled on using a graph based planner based on Voronoi regions to solve
the planning problem. I created the Voronoi graph, pruned edges in collision with the
obstacle map and then build a graph using the `networkx` package. The planner seems
to perform quite well in the simulator.

One thing that I did to improve the interface to the `a_star` method was providing a
method is an input argument that given a node returns all children of that node with
the associated cost of arriving at that child. For grid based planners, this method
uses the `Action` class to generate children nodes. For graph based planners, this
method uses the `networkx` interface to get all of the neighboring nodes and their
associated edge cost. An example of what the grid based method looks like is the
following:

    def grid_get_children(grid, current_node):
        children = []
        for action in valid_actions(grid, current_node):
            da = action.delta
            children.append(((current_node[0] + da[0], current_node[1] + da[1]),
                              action.cost))
        return children

And for the graph based planner this:

    def _graph_get_children(graph, current_node):
        children = []
        for next_node in graph.neighbors(current_node):
            cost = graph.get_edge_data(current_node, next_node)['weight']
            children.append((next_node, cost))
        return children

Each of these methods is then wrapped in a `lambda` and passed into the `a_star` method
as follows:

    get_children = lambda node: _graph_get_children(my_graph, node)
		plan = a_star(get_children, heuristic, start_node, goal_node)

#### 6. Cull waypoints
I used the bresenham python package to perform raytracing in order to prune redundant and
unecessary waypoints from the path. I used the following code to perform this function:

    def _prune_redundant(path, grid):
        if path is not None:
            pruned_path = [path[0]]  # Initialize the pruned path with the first waypoint
            j = 1  # Keep track of the index from the original path
            while j < len(path):
                pi = pruned_path[-1]  # initial point is the last element of the pruned_path
                pe = path[j]          # end point is point 'j' from the original path
								# Run bresenham algorithm to find all grid cells between 'pi' and 'pe'
								# This graphics based algorithm will miss some cells, but with the
								# buffer around the obstacles it shouldn't matter much.
                ray = bresenham(pi[0], pi[1], int(pe[0]), int(pe[1]))
								# Collect the value from the grid for all cells along the line between
								# 'pi' and 'pe'. Take the sum. If no collisions, this should sum to 0.
                redundant = (np.sum([grid[ri,rj] for ri, rj in ray]) == 0)
                # We count a point as 'redundant' if there exists a collision free path
								# between the end of the pruned_path (pi) the that point. Increment
								# th counter and loop again.
                if redundant:
                    j += 1
                else:
                    pruned_path.append(path[j-1])  # Eliminating point 'j' leads to a
										                               # collision, but the point before
																									 # did not, so add it to the pruned_path
																									 # list.
            pruned_path.append(path[-1])  # Make sure the end of the 'path' list is included
						                              # in the pruned_path list.
            return pruned_path
        else:
            return path

The code is commented to explain the algorithm, but basically we walk through the original path
and find the first point that leads to a collision. The point just before this point was
collision free, meaning it is the last non-redundant point in the path. We continue this process
of adding the last collision free point to the list until we get to the end of the path. From
testing in simulation this appears to be a good method of reducing the waypoints in the path. As
a further speed improvement this algorithm could be converted to using a binary search method.

### Execute the flight
#### 1. Does it work?
Yes, it works. I tried flying the quadrotor around manually before executing the motion
planning script to ensure that my modifications to the start location worked successfully.
I then used either Google Maps or the simulator itself to select a geodetic position to apply
as the goal location and ran the planner in order to ensure that planning worked as expected.

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.



