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
## Writeup / README

### Explain the Starter Code

#### 1. `motion_planning.py`

This script is very similar to the backyard flyer script. It uses event driven programming to command the quadrotor to arm, take-off, follow waypoints, land and then disarm.

The key differences between `motion_planning.py` and `backyard_flyer.py` is that
`backyard_flyer.py` follows a fixed set of waypoints in the shape of a box and
`motion_planning.py` has an added state as part of the finite state machine that
allows for planning a path through the urban environment. While the path that is
planned in this skeleton implementation is quite simple, all of the necessary
code is present. One just needs to modify the body of the `plan_path` method.

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
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.



### Execute the flight
#### 1. Does it work?
Yes, it works. I tried flying the quadrotor around manually before executing the motion
planning script to ensure that my modifications to the start location worked successfully.
I then used either Google Maps or the simulator itself to select a geodetic position to apply
as the goal location and ran the planner in order to ensure that planning worked as expected.

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.

# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


