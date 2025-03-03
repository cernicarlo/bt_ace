#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
from rviz_cloud_annotation.msg import LabeledPointCloud2
from pcl_geometric_primitives_detector.msg import LabeledObjInfo 

def pointcloud2_to_xyz_array(pointcloud):
    # Converts a PointCloud2 message into an (N, 3) NumPy array of XYZ points.
    dtype_list = [
        ('x', np.float32), ('y', np.float32), ('z', np.float32)
    ]
    pc_data = np.frombuffer(pointcloud.data, dtype=np.dtype(dtype_list))
    points = np.array([pc_data['x'], pc_data['y'], pc_data['z']]).T
    return points

def calculate_surface_point(cluster_points):
    # Calculate the centroid
    centroid = np.mean(cluster_points, axis=0)
    # Find the highest point on the Z-axis that aligns with centroid X, Y
    surface_point = np.max(cluster_points[
        np.logical_and(
            np.all(cluster_points[:, :2] == centroid[:2], axis=1),
            cluster_points[:, 2] == np.max(cluster_points[:, 2])
        )
    ], axis=0)
    return surface_point

class LabeledPointCloudConverter:
    def __init__(self):
        # Subscriber to /labeled_clouds
        self.subscription = rospy.Subscriber('/labeled_clouds', LabeledPointCloud2, self.callback, queue_size=10)
        
        # Publisher for the converted LabeledObjInfo topic
        self.publisher = rospy.Publisher('/labeled_obj_info', LabeledObjInfo, queue_size=10)

    def callback(self, msg):
        # Parse the input message LabeledPointCloud2
        cloud = msg.cloud
        label_id = msg.label_id
        label_name = msg.label_name
        
        # Convert PointCloud2 to XYZ array
        points = pointcloud2_to_xyz_array(cloud)
        
        # Compute centroid
        centroid = points.mean(axis=0)
        
        # Compute surface point using the provided function
        surface_point = calculate_surface_point(points)
        
        # Create LabeledObjInfo message
        labeled_obj_info = LabeledObjInfo()
        
        # Set centroid
        labeled_obj_info.centroid = Point()
        labeled_obj_info.centroid.x, labeled_obj_info.centroid.y, labeled_obj_info.centroid.z = centroid
        
        # Set surface point
        labeled_obj_info.surface_point = Point()
        labeled_obj_info.surface_point.x, labeled_obj_info.surface_point.y, labeled_obj_info.surface_point.z = surface_point
        
        # Set clustered pointcloud and label
        labeled_obj_info.clustered_pointcloud = cloud
        labeled_obj_info.label = label_name
        
        # Publish the converted message
        self.publisher.publish(labeled_obj_info)

def main(args=None):
    rospy.init_node('labeled_pointcloud_converter')
    node = LabeledPointCloudConverter()
    rospy.spin()

if __name__ == '__main__':
    main()

