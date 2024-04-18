import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
import numpy as np
import airsim 
from pymavlink import mavutil
import argparse
import dronekit
import time

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
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(TargetAltitude)

    while True:
        print("Altitude:", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= TargetAltitude*0.95 :
            print("Reached target altitude")
            break
        time.sleep(1)

arm_and_takeoff(12)

def send_local_ned_velocity(vx,vy,vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
         0,
         0,0,
         mavutil.mavlink.MAV_FRAME_LOCAL_NED,
         0b0000111111000111,
         0, 0, 0,
         vx,vy,vz,
         0,0,0,
         0,0        
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

k_att = 50.0  # Attractive force coefficient
k_rep = 200.0  # Repulsive force coefficient
min_dist = 50.0  # Minimum distance to obstacles
dt = 0.5

def calculate_attractive_force(current_pos, goal_pos):
    
    direction = goal_pos - current_pos
    distance = np.linalg.norm(direction)
    if distance <= 0.0001:
        return np.zeros(3)  
    return (k_att * direction)

def calculate_repulsive_force(current, obstacles):
    repulsive_force = np.zeros(3)

    collision_avoidance_factor = 0.5
    
    for obstacle in obstacles:
        direction = current - obstacle
        distance = np.linalg.norm(direction)
        
        if distance < min_dist:
            force = k_rep * ((1 / distance) - (1 / min_dist)) * (1 / (distance ** (3))) * direction
   
            if distance < 20:
                force *= collision_avoidance_factor
            
            repulsive_force += force
            
    return repulsive_force

def APF_callback(point_cloud):
    goal_position = np.array([-35.3643190000, 149.1638045000, vehicle.location.global_frame.alt])
    max_vel = 10
    copter_pose = client.simGetVehiclePose().position
    while True:
        current_position_local = np.array([copter_pose.x_val, copter_pose.y_val, copter_pose.z_val])
        current_position_global = np.array([vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt])
        #print(current_position_global)
        obstacles = [[p[0], p[1], p[2]] for p in point_cloud2.read_points(point_cloud, skip_nans=True)]
        obstacle_pos = np.array(obstacles)   
        attractive_force = calculate_attractive_force(current_position_global, goal_position)
        repulsive_force = calculate_repulsive_force(current_position_local, obstacle_pos)

        total_force = -(attractive_force + repulsive_force)
        linear_velocities = total_force * max_vel     
        linear_velocities = np.array(linear_velocities)

        current_position = np.array([vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt])
        new_goal_dist = np.linalg.norm(current_position - goal_position)

        send_local_ned_velocity(linear_velocities[0],linear_velocities[1],linear_velocities[2])
        
        print(new_goal_dist)
        if new_goal_dist < 0.0001:  
            break
        print("Target location reached")
        vehicle.mode = dronekit.VehicleMode("LAND")

rospy.init_node('listener', anonymous=True)
subscriber = rospy.Subscriber("RANSAC_topic", PointCloud2, APF_callback)
rospy.spin()


