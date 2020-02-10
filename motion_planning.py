import argparse
import time
import msgpack
import re
from enum import Enum, auto

import numpy as np
from bresenham import bresenham

from planning_utils import a_star, heuristic, create_grid, grid_get_children
from planning_utils import create_2p5d_map, grid_3d_get_children
from prm import ProbabilisticRoadMap
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


## Some helper functions
def _calculate_waypoint_heading(waypoints):

    for k in range(1,len(waypoints)):
        p0 = waypoints[k-1]
        p1 = waypoints[k]
        angle = np.arctan2(p1[1] - p0[1], p1[0] - p0[0])
        print('p0: {}, p1: {}, angle: {}'.format(p0, p1, angle))
        waypoints[k][3] = angle
    waypoints[-1][3] = angle

def _prune_redundant(path, grid):
    # Convert this to use a binary search method.
    if path is not None:
        pruned_path = [path[0]]
        j = 1
        while j < len(path):
            pi = pruned_path[-1]
            pe = path[j]
            #print('Checking {} and {} for collisions.'.format(pi, pe))
            ray = bresenham(pi[0], pi[1], int(pe[0]), int(pe[1]))
            redundant = (np.sum([grid[ri,rj] for ri, rj in ray]) == 0)
            #print('j: {}, ray: {}, redundant: {}'.format(j, ray, redundant))
            if redundant:
                j += 1
            else:
                pruned_path.append(path[j-1])
        pruned_path.append(path[-1])
        return pruned_path
    else:
        return path

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        self._grid_offset = (0, 0)

        self._use_prm = False

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if len(self.waypoints) == 0:
                # Narrower tolerance for terminal waypoint.
                radius = 0.5
            else:
                # Tolerate a larger distance for intermediate waypoints.
                radius = 2.0
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < radius:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        print("arming transition")
        self.arm()
        self.take_control()
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        self.takeoff(self.target_position[2])
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1],
                          self.target_position[2], self.target_position[3])
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.release_control()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def global_to_grid(self, global_position):
         _local_position = global_to_local(global_position, self.global_home)
         return self.local_to_grid(_local_position)

    def local_to_grid(self, local_position):
        return (int(round(local_position[0]) - self._grid_offset[0]),
                int(round(local_position[1]) - self._grid_offset[1]))

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # Read lat0, lon0 from colliders into floating point values
        lat_lon_str = 'lat0 0, lon0 0'  # default in case file read fails.
        with open('colliders.csv') as f:
            lat_lon_str = f.readline()
            f.close()

        # The lat/lon string sort of looks like a comma separate list of key/value
        # pairs, so we'll parse it under that assumption. Obviously the format of
        # the colliders.csv file isn't going to change while completing this project
        # but this adds a bit of robustness to the parsing strategy.
        lat_lon_dict = dict(item.split() for item in lat_lon_str.split(','))
        # Convert to float:
        for k, v in lat_lon_dict.items():
            lat_lon_dict[k] = float(v)
        # set home position to (lon0, lat0, 0)
        self.set_home_position(lat_lon_dict['lon0'],
                               lat_lon_dict['lat0'],
                               0.0)

        # TODO: retrieve current global position

        # We can convert to current local position using global_to_local(), but we don't
        # need to set the local_position, because that is done automatically now that the
        # home position has been set.
        _local_position = global_to_local(self.global_position, self.global_home)
        # Print the temporary variable '_local_position' so we can compare it to the
        # member attribute.
        print('_local_position {0}'.format(_local_position))

        print('global home {0}, position {1}, local position {2}'.format(self.global_home,
                                                                         self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Select a random goal location from Google Maps (using longitude/latitude):
        #goal_location = (-122.398132, 37.796304, 0)  # in front of The Punchline!
        goal_location = (-122.398718, 37.792104, 0)  # temporary closer spot

        if not self._use_prm:
            # Define a grid for a particular altitude and safety margin around obstacles
            map_2p5d, self._grid_offset = create_2p5d_map(data, SAFETY_DISTANCE)
            print("North offset = {0}, east offset = {1}".format(self._grid_offset[0],
                                                                 self._grid_offset[1]))

            # Define starting point on the grid. The 'local_position' is relative to the
            # grid center, so we want to set the starting position on the grid equal to
            # our current position, offset by the grid offset.
            grid_start = self.local_to_grid(self.local_position)

            # Convert the lat/lon goal location into a grid location:
            grid_goal = self.global_to_grid(goal_location)

            #grid_goal = self.local_to_grid((-30, 60))
            #grid_goal = tuple(np.random.randint(min(map_2p5d.shape), size=(1, 2)).tolist()[0])

            # Make sure that the start and goal locations are achievable!
            height_start = map_2p5d[grid_start]
            height_goal = map_2p5d[grid_goal]
            print('start height: {}, goal height: {}'.format(height_start, height_goal))

            # Find the maximum height between the target height & the start/goal height
            height = max(max(height_start, height_goal), TARGET_ALTITUDE)

            # Generate the grid for the given height.
            grid = map_2p5d > height

            get_children = lambda node: grid_get_children(grid, node)

            # Run A* to find a path from start to goal
            print('Local Start and Goal: ', grid_start, grid_goal)
            path, _ = a_star(get_children, heuristic, grid_start, grid_goal)

            print('Found a path with {} nodes.'.format(len(path)))

            # Prune path to minimize number of waypoints
            pruned_path = _prune_redundant(path, grid)
            print(pruned_path)
            print('pruned path has {} waypoints.'.format(len(pruned_path)))

            path = pruned_path

            # Convert path to waypoints
            waypoints = [[p[0] + self._grid_offset[0], p[1] + self._grid_offset[1],
                          height, 0] for p in path]

            print(waypoints)
            _calculate_waypoint_heading(waypoints)
            print(waypoints)
        else:  # Use the PRM to plan a path.

            local_start = tuple(self.local_position.tolist());
            local_goal = tuple(global_to_local(goal_location, self.global_home).tolist())
            print('start: {}, goal: {}'.format(local_start, local_goal))

            prm = ProbabilisticRoadMap(data, 500, TARGET_ALTITUDE, TARGET_ALTITUDE)

            path = prm.plan_path(local_start, local_goal)

            waypoints = [[p[0], p[1], TARGET_ALTITUDE, 0] for p in path]
            print(waypoints)
            _calculate_waypoint_heading(waypoints)
            print(waypoints)

        # Set self.waypoints
        self.waypoints = waypoints
        # Send waypoints to sim (this is just for visualization of waypoints)
        # This is wrapped in a try/except block because the simulator sometimes barfs
        # on this call which is annoying.
        try:
            self.send_waypoints()
        except AttributeError:
            pass

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
