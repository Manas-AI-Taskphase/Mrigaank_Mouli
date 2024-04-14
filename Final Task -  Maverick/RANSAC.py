import airsim
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2 , PointField
from geometry_msgs.msg import Point32
import pcl
from std_msgs.msg import Header
import numpy as np


client = airsim.MultirotorClient()


rospy.init_node('RANSAC_publisher', anonymous=True)
pub = rospy.Publisher('RANSAC_topic', PointCloud2, queue_size=10)
rate = rospy.Rate(10)  

def ransac_plane_segmentation(points, distance_threshold=0.5, max_iterations=50000, probability=0.99):
    num_points = len(points)
    sample_size = 3
    best_inliers = None
    best_outliers = None
    best_plane_params = None
    
    for _ in range(max_iterations):
        sample_indices = np.random.choice(num_points, size=sample_size, replace=True)
        sample_points = points[sample_indices]
        

        _, _, vh = np.linalg.svd(sample_points - sample_points.mean(axis=0))
        plane_normal = vh[-1]
        d = -np.dot(plane_normal, sample_points[0])
        plane_params = np.concatenate((plane_normal, [d]))
        
        distances = np.abs(np.dot(points, plane_normal) + d) / np.linalg.norm(plane_normal)
        
        inlier_indices = np.where(distances <= distance_threshold)[0]
        outlier_indices = np.where(distances > distance_threshold)[0]
        
        if best_inliers is None or len(inlier_indices) > len(best_inliers):
            best_inliers = inlier_indices
            best_outliers = outlier_indices
            best_plane_params = plane_params
            
        inlier_ratio = len(inlier_indices) / num_points
        if inlier_ratio >= probability:
            break
    

    inliers = points[best_inliers]
    outliers = points[best_outliers]

    return inliers, outliers, best_plane_params


while not rospy.is_shutdown():
    point_cloud = client.getLidarData().point_cloud

    x_positions = point_cloud
    points = []
    for i in range(0,int(len(x_positions) / 3)):
        
        x = x_positions[3*i] * -1
        y = x_positions[3*i + 1] * -1
        z = x_positions[3*i + 2] * -1
        point = [x,y,z]
        points.extend([point])

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1)]
    
    header = Header()
    header.frame_id = "map"

    cloud = point_cloud2.create_cloud(header,fields,points)
    obstacles = [[p[0], p[1], p[2]] for p in point_cloud2.read_points(cloud, skip_nans=True)]
    obstacle_pos = np.array(obstacles)   

    inliers, outliers,_= ransac_plane_segmentation(obstacle_pos, distance_threshold=0.5, max_iterations=1000)
    
    header = Header()
    header.frame_id = 'map'
    
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    
    msg = point_cloud2.create_cloud(header, fields, outliers)
    pub.publish(msg)

rate.sleep()

