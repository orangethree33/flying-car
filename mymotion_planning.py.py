import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
from queue import PriorityQueue

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

     # Run A* to find a path from start to goal

class Action(Enum):
    WEST=(0,-1,1)
    EAST=(0,1,-1)
    SORTH=(1,0,1)
    NORTH=(-1,0,1)
    SORTH_WEST=(1,-1,np.sqrt(2))
    SORTH_EAST=(1,1,np.sqrt(2))
    NORTH_WEST=(-1,1,np.sqrt(2))
    NORTH_EAST=(-1,-1,np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0],self.value[1])

def valid_actions(grid,current_node):
        
        valid=[Action.WEST,Action.EAST,Action.NORTH,Action.SORTH,Action.SORTH_WEST,Action.SORTH_EAST,Action.NORTH_WEST,Action.NORTH_EAST]
        n,m = grid.shape[0]-1,grid.shape[1]-1
        x,y=current_node

        if x-1<0 or grid[x-1,y]==1:
            valid_actions.remove(Action.NORTH)
        if x+1>n or grid[x+1,y]==1:
            valid_actions.remove(Action.SORTH)
        if y-1<0 or grid[x,y-1]==1:
            valid_actions.remove(Action.WEST)
        if y+1>m or grid[x,y+1]==1:
            valid_actions.remove(Action.EAST)   

        if (x-1<0 and y-1<0) or grid[x-1,y-1]==1:
            valid_actions.remove(Action.NORTH_WEST)
        if (x-1<0 and y+1>m) or grid[x-1,y-1]==1:
            valid_actions.remove(Action.NORTH_EAST)
        if (x+1<0 and y-1<0) or grid[x-1,y-1]==1:
            valid_actions.remove(Action.SOUTH_WEST)
        if (x+1<n and y+1>m) or grid[x-1,y-1]==1:
            valid_actions.remove(Action.SOUTH_EAST) 
        return valid   

def heuristic(position,global_position):

        
        h=np.linalg.norm(np.array(position)-np.array(global_position))
        return h   

def a_star(grid,heuristic,grid_start,grid_goal):
        path=[]
        queue=PriorityQueue()
        queue.put((0,grid_start))
        visited=set(grid_start)

        branch={}
        found=False


        while not queue.empty():
            item=queue.get()
            #print('item=',item[1])
            current_cost=item[0]
            current_node=item[1]
            #print('cu=',current_node,' goal=',grid_goal)
            if current_node==grid_goal:
                print('Found a path.')
                found=True
                break

            else:
                for action in valid_actions(grid,current_node):
                    #get the tuple representation
                    da = action.delta
                    cost=action.cost
                    next_node=(current_node[0]+da[0],current_node[1]+da[1])
                    new_cost=current_cost+cost+heuristic(next_node,grid_goal)
                
                    if next_node not in visited:
                        visited.add(next_node)
                        queue.put((new_cost,next_node))
                        branch[next_node]=(new_cost,current_node,action)
                        n=next_node
                        

            #path=[]
            #path_cost=0
        if found:
                #retrace steps
                #path=[]
                n=grid_goal
                path_cost=branch[n][0]
                path.append(grid_goal)
                while branch[n][1] != grid_start:
                    path.append(branch[n][1])
                    n=branch[n][1]
                    #print('cu=',branch[n][1])
                path.append(branch[n][1])
                print('path=',path)
                    
        return path[::-1],path_cost

                
        
def point(p):
            return np.array([p[0],p[1],1.]).reshape(1,-1)
    
def collinearity_check(p1, p2, p3, epsilon=1e-6):   
            m = np.concatenate((p1, p2, p3), 0)
            det = np.linalg.det(m)
            return abs(det) < epsilon

def prune_path(path):
        pruned_path=[p for p in path]
        i=0;
        while i<len(pruned_path)-2:
            p1=point(pruned_path[i])
            p2=point(pruned_path[i+1])
            p3=point(pruned_path[i+2])
            if collinearity_check(p1,p2,p3):
                pruned_path.remove(pruned_path[i+1])
            else:
                i+=1
        return pruned_path
        
       


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
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
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        
        # TODO: set home position to (lat0, lon0, 0)

        # TODO: retrieve current global position
 
        # TODO: convert to current local position using global_to_local()
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        
        # Set goal as some arbitrary position on the grid
        grid_goal = (-north_offset + 40, -east_offset + 30)
        #or other location such as 10m sorth and 20 west of map
        # TODO: adapt to set goal as latitude / longitude position and convert

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)

        print('Local Start and Goal: ', grid_start, grid_goal)

        path,cost=a_star(grid,heuristic,grid_start,grid_goal)
        #path(path,cost)
        print ('1 ',len(path),cost)

        

        # Convert path to waypoints
        pruned_path=prune_path(path)
        print (len(pruned_path))

        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypointsi
        self.waypoints = waypoints
        
        self.send_waypoints()

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
