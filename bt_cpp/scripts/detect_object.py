import numpy as np
from geometry_msgs.msg import PointStamped
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2

# FIXME: the whole script is just for proof of concept

# Sphere
obj_position_x = 3.0
obj_position_y = 4.0
obj_position_z = 5.0

# LUMA station
# obj_position_x = -10.0
# obj_position_y = 5.0
# obj_position_z = 6.4

pos_auv_close_obj_x = 0 # adjust to 0
pos_auv_close_obj_y = 0
pos_auv_close_obj_z = 3

object_position_pub = rospy.Publisher('object_pose', PointStamped, queue_size=10)

def detect_object_in_front(pointcloud_msg):
    # FIXME: develop it properly
    # Define detection range
    x_range = (0, 5)  # 0 to 5 meters in front of the camera
    y_range = (-5, 5)  # Adjust lateral range as necessary
    z_range = (-4, 4)  # From -3 to 3 meters in height

    # Extract points from the PointCloud2 message
    for point in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point

        # Check if point is within the specified range
        if x_range[0] <= x <= x_range[1] and y_range[0] <= y <= y_range[1] and z_range[0] <= z <= z_range[1]:
            # rospy.loginfo(f"Object detected at position x={x}, y={y}, z={z}")
            return True  # Object detected

    return False  # No object detected

def pointcloud_callback(msg):
    condition = False
    publish_object_position(condition)


def publish_object_position(is_condition_achieved):
    object_position = PointStamped()
    object_position.header.stamp = rospy.Time.now()
    object_position.header.frame_id = "world_ned"  # Use the appropriate frame
    if is_condition_achieved:
        # rospy.loginfo("Object detected!")
        object_position.point.x = obj_position_x
        object_position.point.y = obj_position_y
        object_position.point.z = obj_position_z
        
    else:
        object_position.point.x = np.nan
        object_position.point.y = np.nan
        object_position.point.z = np.nan
    # Publish to /object_position
    object_position_pub.publish(object_position)


def is_auv_close_to_object(odom_msg):
    # FIXME: this is a mock, only working with priviledged info!
    tol_x = 15 # adjust to 3
    tol_y = 15
    tol_z = 15
    tol_yaw = 10
    relative_path_orientation = 0.7

    is_x_close = abs(odom_msg.pose.pose.position.x - pos_auv_close_obj_x) < tol_x
    is_y_close = abs(odom_msg.pose.pose.position.y - pos_auv_close_obj_y) < tol_y
    is_z_close = abs(odom_msg.pose.pose.position.z - pos_auv_close_obj_z) < tol_z
    is_yaw_aiming = abs(odom_msg.pose.pose.orientation.z - relative_path_orientation) < tol_yaw
    # is_yaw_aiming = True

    return (is_x_close & is_y_close & is_z_close & is_yaw_aiming)

def odom_callback(msg):
    condition = is_auv_close_to_object(msg)
    if condition:
        rospy.loginfo(f"AUV in x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}, z={msg.pose.pose.position.z} close to object!")
        # TODO: publish only if auv started path (waypoint >= 2)
        publish_object_position(condition)

def pointcloud_listener():
    rospy.init_node('object_detection_node', anonymous=True)
    # FIXME: implement it properly
    rospy.Subscriber("/girona1000/depth_cam/pointcloud", PointCloud2, pointcloud_callback)

    # TODO: implement the real object detection
    rospy.Subscriber("/girona1000/navigator/odometry", Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    pointcloud_listener()
