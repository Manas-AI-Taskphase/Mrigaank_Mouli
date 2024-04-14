import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
import numpy as np
import airsim 
import math
import argparse
import dronekit
import time
from dronekit import connect, LocationGlobalRelative, VehicleMode
from heapq import heappop, heappush

client = airsim.MultirotorClient()
client.confirmConnection()


def connectCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    
    connection_string = args.connect
    
    vehicle = dronekit.connect(connection_string, wait_ready=True)
    
    return vehicle

vehicle = connectCopter()

def arm_and_takeoff(TargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = dronekit.VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(TargetAltitude)

    while True:
        print("Altitude:", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= TargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

arm_and_takeoff(12)


class State:
    def __init__(self, latitude, longitude, altitude, local_x, local_y, local_z, cost=float('inf'), parent=None):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.local_x = local_x
        self.local_y = local_y
        self.local_z = local_z
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost

copter_pose = client.simGetVehiclePose().position
def linear_interpolation(start, end, num_waypoints):

    x0, y0, z0 = start
    x1, y1, z1 = end
    waypoints = []
    x_points = np.linspace(x0, x1, num_waypoints)
    y_points = np.linspace(y0, y1, num_waypoints)
    z_points = np.linspace(z0, z1, num_waypoints)
    

    for x, y, z in zip(x_points, y_points, z_points):
        waypoints.append((x,y,z))

    waypoints.append((x1,y1,z1))

    return waypoints

start_position = np.array((vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,copter_pose.z_val))
goal_position = np.array((-35.3643190000,149.1638045000,copter_pose.z_val)) 


num_waypoints = 5

waypoints = linear_interpolation(start_position, goal_position, num_waypoints)
for point in waypoints:
    print (point)

#waypoints = [State(latitude=lat, longitude=lon, altitude=alt, local_x=None, local_y=None, local_z=None) for lat, lon, alt in positions]

def heuristic(state, goal):
    lat1, lon1, alt1 = state.latitude, state.longitude, state.altitude
    lat2, lon2, alt2 = goal.latitude, goal.longitude, goal.altitude
    return math.sqrt((lat2 - lat1) ** 2 + (lon2 - lon1) ** 2 + (alt2 - alt1) ** 2)

def distance(point1, point2):
    x1 = point1.latitude
    x2 = point2.latitude
    y1 = point1.longitude
    y2 = point2.longitude
    z1 = point1.altitude
    z2 = point2.altitude
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

def get_neighbors(state, obstacles):
    lat, lon, alt = state.latitude, state.longitude, state.altitude
    x, y, z = state.local_x, state.local_y, state.local_z
    movements = []
    for i in range(-1, 2):
        for j in range(-1, 2):
            for k in range(-1, 2):
                movements.append([i, j, k])
    neighbor_count = 0
    neighbors = []
    for dx, dy, dz in movements:
        new_lat, new_lon, new_alt = lat + dx * 0.0001, lon + dy * 0.00001, alt + dz * 0.00001
        new_x, new_y, new_z = x + dx, y + dy , z + dz 

        if new_alt>15 or new_alt<10:
            continue
        #if new_lat>35.3643190000 or new_lat<(-35.3643190000):
         #   continue

        new_state = State(latitude=new_lat, longitude=new_lon, altitude=new_alt, local_x=new_x, local_y=new_y, local_z=new_z, cost=float('inf'), parent=None)
        
        if is_valid_position(new_state, obstacles):
            neighbor_count +=1
            neighbors.append(new_state)
        print (f"No of Neighbours:{neighbor_count}")        
    return neighbors

def is_valid_position(state, obstacles):
    x = state.local_x
    y = state.local_y
    z = state.local_z
    coords = np.array([x, y, z])
    for obstacle in obstacles:
        if np.allclose(coords,obstacle,atol=1e-2):
            return False
    return True


def astar(start, goal, obstacles):
    start.cost = heuristic(start,goal)
    open_set = [(start.cost, start)]  # Priority queue (cost, state)
    closed_set = set() 
    counter = 0
    while open_set:
        print(f"No of Iterations:{counter}")
 
        if open_set:
            _, current_state = heappop(open_set)
        else:
            return None  
        print(current_state.cost)
        current_pos= np.array((current_state.latitude,current_state.longitude,current_state.altitude))
        goal_pos = np.array((goal.latitude,goal.longitude,goal.altitude))
        print(f"Current:{current_pos}")
        print(f"Goal:{goal_pos}")
        if np.allclose(goal_pos,current_pos,atol=1e-5):
            return reconstruct_path(current_state)
        counter +=1
        closed_set.add(current_state)

        for neighbor_state in get_neighbors(current_state, obstacles):
            if neighbor_state in closed_set:
                continue

            tentative_cost = current_state.cost + distance(current_state, neighbor_state) + heuristic(neighbor_state, goal)
            print(tentative_cost)
            if tentative_cost < neighbor_state.cost:
                neighbor_state.cost = tentative_cost
                neighbor_state.parent = current_state
                if (tentative_cost, neighbor_state) not in open_set:
                    print(f"Neighbour{neighbor_state.latitude,neighbor_state.longitude,neighbor_state.altitude}")
                    heappush(open_set, (tentative_cost, neighbor_state))            
    return None  # No path found

def reconstruct_path(state):
    path = []
    while state:
        path.append((state.latitude, state.longitude, state.altitude))
        state = state.parent
    return list(reversed(path))


rospy.init_node('listener', anonymous=True)

start = State(latitude=vehicle.location.global_frame.lat,longitude=vehicle.location.global_frame.lon,altitude=copter_pose.z_val,local_x=copter_pose.x_val,local_y=copter_pose.y_val,local_z=copter_pose.z_val)
for i in range(1,num_waypoints+1):
    lat,lon,alt = waypoints[i]
    intermediate = State(latitude=lat,longitude=lon,altitude=copter_pose.z_val,local_x=None,local_y=None,local_z=None)
    point_cloud = rospy.wait_for_message("RANSAC_topic", PointCloud2, timeout=None)
    obstacle_pos = [[p[0], p[1], p[2]] for p in point_cloud2.read_points(point_cloud, skip_nans=True)]
    obstacles = np.array(obstacle_pos)
    print(f"Obstacles:{obstacles}")
    path = astar(start,intermediate, obstacles)

    if path is None:
        print("No path found")
    else:
        print("Computed Path:")
        for position in path:
            print(position)

        for position in path:
                x, y, z = position
                vehicle.simple_goto(LocationGlobalRelative(x, y, z))  
    current = State(latitude=vehicle.location.global_frame.lat,longitude=vehicle.location.global_frame.lon,altitude=copter_pose.z_val,local_x=copter_pose.x_val,local_y=copter_pose.y_val,local_z=copter_pose.z_val)    

    start = current    
    current_pos = np.array((vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,copter_pose.z_val))   
    if np.allclose(current_pos,goal_position,atol=1e-5):
        break

print("Target Reached")
vehicle.mode = VehicleMode("LAND")