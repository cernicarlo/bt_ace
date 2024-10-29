#!/usr/bin/env python

import rospy
import open3d as o3d
import numpy as np
from scipy.spatial import Delaunay
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
import struct
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion

class MyOpen3DNode:
    def __init__(self):
        rospy.init_node('my_open3d_node', anonymous=True)

        # Initialize a TF listener to get the transforms
        self.tf_listener = TransformListener()

        # Subscriber to get the incoming PointCloud2 messages
        self.subscription = rospy.Subscriber(
            'input_pointcloud',  # Input topic name
            PointCloud2,
            self.point_cloud_callback,
            queue_size=10
        )

        # Publisher to send the reconstructed PointCloud2 messages
        self.publisher = rospy.Publisher(
            'luma_computer/compressed_cloud',  # Output topic name
            PointCloud2,
            queue_size=10
        )

        rospy.spin()

    def point_cloud_callback(self, msg):
        try:
            # Lookup the transformation for the optical modems
            self.tf_listener.waitForTransform('base_link', 'luma_girona', rospy.Time(0), rospy.Duration(4.0))
            (trans1, rot1) = self.tf_listener.lookupTransform('base_link', 'luma_girona', rospy.Time(0))
            self.tf_listener.waitForTransform('base_link', 'luma_computer', rospy.Time(0), rospy.Duration(4.0))
            (trans2, rot2) = self.tf_listener.lookupTransform('base_link', 'luma_computer', rospy.Time(0))

            optical_modem_tf_1 = np.array(trans1)  # Position of luma_girona
            optical_modem_tf_2 = np.array(trans2)  # Position of luma_robot

            # Convert rotation (quaternion) to Euler angles
            euler1 = euler_from_quaternion(rot1)
            euler2 = euler_from_quaternion(rot2)

            # Example water condition value; replace this with actual data
            water_condition = 0.5  # Placeholder for water condition value (between 0 and 1)

            # Check line of sight before processing point cloud
            if self.check_line_of_sight(optical_modem_tf_1, optical_modem_tf_2, euler1, euler2, water_condition):
                # Step 1: Convert PointCloud2 to Open3D PointCloud
                point_cloud = self.pointcloud2_to_open3d(msg)

                # Step 2: Calculate normals for the point cloud
                point_cloud_with_normals = self.calculate_normals(point_cloud)

                # Step 3: Convert point cloud to mesh using Delaunay triangulation
                mesh = self.point_cloud_to_mesh(point_cloud_with_normals)

                # Step 4: Clean up the mesh by removing long edges
                distance_threshold = 0.02  # Adjust this value based on your data
                cleaned_mesh = self.clean_mesh(mesh, distance_threshold)

                # Step 5: Compress the mesh (quadric edge collapse simplification)
                simplified_mesh = self.compress_mesh(cleaned_mesh, target_triangle_count=200)

                # Step 6: Reconstruct point cloud from compressed mesh
                reconstructed_point_cloud = self.mesh_to_point_cloud(simplified_mesh)

                # Step 7: Convert Open3D PointCloud back to PointCloud2
                output_msg = self.open3d_to_pointcloud2(reconstructed_point_cloud, msg.header)

                # Publish the reconstructed point cloud
                self.publisher.publish(output_msg)
            else:
                rospy.loginfo("Optical modems cannot see each other. Point cloud processing skipped.")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF lookup error: {e}")

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
        angle_diff = abs(yaw1 - yaw2)
        if angle_diff > np.pi / 4:  # For example, 45 degrees
            return False

        return True

    def calculate_normals(self, point_cloud):
        point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
        return point_cloud

    def point_cloud_to_mesh(self, point_cloud):
        points = np.asarray(point_cloud.points)
        tri = Delaunay(points[:, :2])  # Using only x and y coordinates for 2D triangulation
        triangles = tri.simplices
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(points)
        mesh.triangles = o3d.utility.Vector3iVector(triangles)
        return mesh

    def clean_mesh(self, mesh, distance_threshold):
        vertices = np.asarray(mesh.vertices)
        triangles = np.asarray(mesh.triangles)
        cleaned_triangles = []

        for tri in triangles:
            v0, v1, v2 = vertices[tri]
            edge_lengths = [
                np.linalg.norm(v0 - v1),
                np.linalg.norm(v1 - v2),
                np.linalg.norm(v2 - v0)
            ]
            if max(edge_lengths) < distance_threshold:
                cleaned_triangles.append(tri)

        mesh.triangles = o3d.utility.Vector3iVector(cleaned_triangles)
        return mesh

    def compress_mesh(self, mesh, target_triangle_count):
        simplified_mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=target_triangle_count)
        return simplified_mesh

    def mesh_to_point_cloud(self, mesh):
        return mesh.sample_points_poisson_disk(number_of_points=500)  # Adjust number_of_points

    def pointcloud2_to_open3d(self, msg):
        # Convert ROS PointCloud2 message to Open3D point cloud
        points = []
        for point in self.read_points(msg):
            points.append(point)

        open3d_pc = o3d.geometry.PointCloud()
        open3d_pc.points = o3d.utility.Vector3dVector(np.array(points))
        return open3d_pc

    def read_points(self, cloud):
        # Generator to parse PointCloud2 messages into XYZ points
        fmt = 'fff'  # Format for 3 floats (x, y, z)
        width, height, point_step, row_step, data = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data
        unpack_from = struct.Struct(fmt).unpack_from
        for v in range(height):
            offset = row_step * v
            for u in range(width):
                point = unpack_from(data, offset + u * point_step)
                yield point

    def open3d_to_pointcloud2(self, open3d_pc, header):
        # Convert Open3D point cloud back to ROS PointCloud2 message
        points = np.asarray(open3d_pc.points, dtype=np.float32)

        # Create a PointCloud2 message
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

if __name__ == '__main__':
    try:
        MyOpen3DNode()
    except rospy.ROSInterruptException:
        pass

