#!/usr/bin/env python

import rospy
import open3d as o3d
import os
import numpy as np
from scipy.spatial import Delaunay
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
import struct
from pcl_geometric_primitives_detector.msg import SimulationInfo
import tf
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool

# import ros_numpy  # Useful for converting PointCloud2 data to numpy

class MyOpen3DNode:
    def __init__(self):
        rospy.init_node('my_open3d_node', anonymous=True)

        # Timer to update cluster subscribers every second
        self.timer_update_cluster = rospy.Timer(rospy.Duration(0.5), self.update_cluster_subscribers)

        # Dictionary to hold subscribers to avoid re-subscribing to the same topics
        self.subscribers = {}

        self.simulation_info_subscriber = rospy.Subscriber(
            '/stonefish_simulator/simulation_info',
            SimulationInfo,  # Adjust to the actual message type
            self.simulation_info_callback,
            queue_size=10
        )
        # Define the publisher for the luma detection topic
        self.is_luma_detected_publisher = rospy.Publisher("/is_luma_detected", Bool, queue_size=10)
        self.is_luma_detected = False
        self.tf_listener = tf.TransformListener()
        # Timer to run check_vlc_line_of_sight periodically (e.g., every 1 second)
        self.timer_check_line_of_sight = rospy.Timer(rospy.Duration(0.5), self.check_vlc_line_of_sight)
        
        self.water_type = 0.2

    def update_cluster_subscribers(self, event=None):
        # Get a list of all current topic names and types
        current_topics = rospy.get_published_topics()

        # Filter the topics that match the pattern '/cluster_'
        cluster_topics = [topic for topic, _ in current_topics if topic.startswith('/cluster_')]
        print(f"cluster_topics: {cluster_topics}")

        for topic in cluster_topics:
            # Subscribe to the new cluster topic if not already subscribed
            if topic not in self.subscribers:
                self.subscribers[topic] = rospy.Subscriber(
                    topic, PointCloud2, self.point_cloud_callback, callback_args=topic
                )
                rospy.loginfo(f"Subscribed to {topic}")

    def simulation_info_callback(self, msg):
        # Extract the water_type value from the SimulationInfo message
        self.water_type = msg.water_type

    def check_vlc_line_of_sight(self, event=None):
        try:
            # Lookup the transformation for the optical modems
            self.tf_listener.waitForTransform('world_ned', 'girona1000/vlc_link', rospy.Time(0), rospy.Duration(4.0))
            (trans1, rot1) = self.tf_listener.lookupTransform('world_ned', 'girona1000/vlc_link', rospy.Time(0))
            self.tf_listener.waitForTransform('world_ned', 'vlc_station', rospy.Time(0), rospy.Duration(4.0))
            (trans2, rot2) = self.tf_listener.lookupTransform('world_ned', 'vlc_station', rospy.Time(0))

            optical_modem_tf_1 = np.array(trans1)  # Position of luma_girona
            optical_modem_tf_2 = np.array(trans2)  # Position of luma_computer

            # Convert rotation (quaternion) to Euler angles
            euler1 = euler_from_quaternion(rot1)
            euler2 = euler_from_quaternion(rot2)
            
            if self.check_line_of_sight(optical_modem_tf_1, optical_modem_tf_2, euler1, euler2, self.water_type):
                rospy.loginfo(f"LUMA detected!")
                self.is_luma_detected = True
            else:
                rospy.loginfo(f"LUMA NOT detected!")
                self.is_luma_detected = False
            self.is_luma_detected_publisher.publish(Bool(data=self.is_luma_detected))


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF lookup error: {e}")



    def point_cloud_callback(self, msg, topic_name):
        # Check line of sight before processing point cloud
        if self.is_luma_detected:  
        
            # Step 1: Convert PointCloud2 to Open3D PointCloud
            point_cloud = self.pointcloud2_to_open3d(msg)
            # Check if conversion was successful
            if point_cloud is None:
                rospy.logwarn(f"Skipping processing for topic {topic_name} due to empty point cloud data.")
                return
            # Step 2: Convert point cloud to mesh using Delaunay triangulation
            mesh = self.point_cloud_to_mesh(point_cloud)
            # Step 3: Compress the mesh (quadric edge collapse simplification)
            simplified_mesh = self.compress_mesh(mesh, target_triangle_count=500)
            # Step 4: Reconstruct point cloud from compressed mesh
            reconstructed_point_cloud = self.mesh_to_point_cloud(simplified_mesh)
            # Step 5: Convert Open3D PointCloud back to ROS PointCloud2
            output_msg = self.open3d_to_pointcloud2(reconstructed_point_cloud, msg.header)
            # Get the corresponding compressed topic
            compressed_topic = topic_name.replace('/cluster_', 'vlc_computer/compressed_cloud_')
            # Create publisher dynamically for each compressed topic
            publisher = rospy.Publisher(compressed_topic, PointCloud2, queue_size=10) 
            # Publish to the compressed topic
            publisher.publish(output_msg)
            rospy.loginfo(f"Processed and published compressed point cloud on {compressed_topic}")
        else:
            rospy.loginfo("Optical modems cannot see each other. Point cloud processing skipped.")


    def check_line_of_sight(self, modem1_pos, modem2_pos, euler1, euler2, water_condition):
        # Calculate the distance between the two modems
        distance = np.linalg.norm(modem1_pos - modem2_pos)

        # Determine visibility threshold based on water condition
        visibility_threshold = 10.0 * (1 - water_condition)  # Example: max distance decreases as water condition increases

        # Check if distance is within the visibility threshold
        if distance > visibility_threshold:
            return False

        yaw1 = euler1[2]  # Z-axis rotation for luma_girona
        yaw2 = euler2[2]  # Z-axis rotation for luma_computer

        # Check if the angles of both modems are approximately aligned
        yaw1 += 1.5707963 # yaw of girona looking from reverse
        angle_diff = abs(yaw1 - yaw2)
        if angle_diff > np.pi / 4:  # For example, 45 degrees
            print(f"angle_diff({angle_diff}) = abs( yaw1({yaw1}) - yaw({yaw2})) > np.pi / 4({np.pi / 4})")
            return False

        return True



    def point_cloud_to_mesh(self, point_cloud):
        voxel_size = 0.02  # Adjust voxel size for balance between speed and detail
        downsampled_point_cloud = point_cloud.voxel_down_sample(voxel_size=voxel_size)
        downsampled_point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
        radii = [0.05, 0.1, 0.2]
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            downsampled_point_cloud, o3d.utility.DoubleVector(radii)
        )
        return mesh

    def compress_mesh(self, mesh, target_triangle_count):
        simplified_mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=target_triangle_count)
        return simplified_mesh

    def mesh_to_point_cloud(self, mesh):
        return mesh.sample_points_poisson_disk(number_of_points=500)

    def pointcloud2_to_open3d(self, msg):
        points = []
        for point in self.read_points(msg):
            points.append(point)
        
        if not points:
            rospy.logwarn("Received empty PointCloud2 message. Skipping conversion.")
            return None

        open3d_pc = o3d.geometry.PointCloud()
        open3d_pc.points = o3d.utility.Vector3dVector(np.array(points))
        return open3d_pc

    def read_points(self, cloud):
        fmt = 'fff'  # Format for 3 floats (x, y, z)
        width, height, point_step, row_step, data = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data
        unpack_from = struct.Struct(fmt).unpack_from
        for v in range(height):
            offset = row_step * v
            for u in range(width):
                point = unpack_from(data, offset + u * point_step)
                yield point

    def open3d_to_pointcloud2(self, open3d_pc, header):
        points = np.asarray(open3d_pc.points, dtype=np.float32)

        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.is_dense = True
        cloud_msg.is_bigendian = False
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.point_step = 12  # 4 bytes per float
        cloud_msg.row_step = cloud_msg.point_step * len(points)
        cloud_msg.data = np.asarray(points, np.float32).tobytes()
        return cloud_msg

def main():
    node = MyOpen3DNode()
    rospy.spin()

if __name__ == '__main__':
    main()

