import airsim
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2,PointField
from geometry_msgs.msg import Point32
from std_msgs.msg import Header

# script that gets the current point cloud and publishes it to rviz modify this accordingly 

client = airsim.MultirotorClient()

point_cloud = client.getLidarData().point_cloud

print(len(point_cloud))


rospy.init_node('point_cloud_publisher', anonymous=True)
pub = rospy.Publisher('point_cloud_topic', PointCloud2, queue_size=10)
rate = rospy.Rate(10)  # 10hz

while not rospy.is_shutdown():
    # Create a new PointCloud message
    point_cloud = client.getLidarData().point_cloud

    x_positions = point_cloud
    print(len(x_positions))
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

    pub.publish(cloud)

rate.sleep()
