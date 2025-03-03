#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse  # Import the Trigger service
import struct
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from sklearn.neighbors import NearestNeighbors
from std_msgs.msg import Int32

class PointCloudCluster:
    def __init__(self):
        rospy.init_node('point_cloud_cluster_node', anonymous=True)

        # Subscriber to PointCloud2 topic
        self.pointcloud_subscriber = rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, self.pointcloud_callback)

        # A dictionary to hold publishers for dynamic topics (clusters)and relative msg
        self.clustered_point_publishers = {}
        self.clustered_point_msg = {}

        # Variables to store clusters and cluster labels
        self.clusters = None
        self.cluster_labels = None
        self.clustering_done = False  # Flag to ensure clustering is only done once
        self.latest_pointcloud = None  # Store the latest point cloud

        # Create the trigger service
        self.trigger_service = rospy.Service('clustering', Trigger, self.trigger_clustering)

        # publish hoe many clusters were detected
        self.num_clusters_publisher = rospy.Publisher('/num_clusters', Int32, queue_size=10)

        self.publish_clusters_timer = rospy.Timer(rospy.Duration(1.0), self.publish_clusters)
        rospy.loginfo('PointCloud Clustering Node is ready and waiting for trigger.')

    def pointcloud_callback(self, msg):
        """Store the PointCloud2 message for later use."""
        self.latest_pointcloud = msg
        # print(f"pc: {msg}")

    def trigger_clustering(self, req):
        """Service callback to start clustering when triggered."""
        if self.latest_pointcloud is None:
            rospy.logwarn("No point cloud data available yet.")
            return TriggerResponse(success=False, message="No point cloud data available.")

        if self.clustering_done:
            rospy.loginfo("Clustering has already been done. Reusing the previous result.")
            return TriggerResponse(success=True, message="Clustering already done.")

        rospy.loginfo('Clustering triggered.')

        # Unpack the PointCloud2 message
        points = self._parse_point_cloud(self.latest_pointcloud)

        # Scale the data
        scaler = StandardScaler()
        points_scaled = scaler.fit_transform(points)

        # Find optimal `eps` for DBSCAN
        optimal_eps = self._find_optimal_eps(points_scaled)

        # Perform DBSCAN clustering
        dbscan = DBSCAN(eps=optimal_eps, min_samples=5)
        self.cluster_labels = dbscan.fit_predict(points_scaled)

        # Store the clusters, ignoring noise points labeled as -1
        self.clusters = []
        unique_labels = set(self.cluster_labels) - {-1}  # Exclude noise (-1)
        for cluster_id in unique_labels:
            cluster_points = points[self.cluster_labels == cluster_id]
            self.clusters.append(cluster_points)

            # Create a unique topic for each cluster
            topic_name = f'/cluster_{cluster_id}'
            # Create the PointCloud2 message for the cluster
            cluster_pc2_msg = self.create_pointcloud2(cluster_points)
            # If we don't have a publisher for this topic, create one
            if topic_name not in self.clustered_point_publishers:
                self.clustered_point_publishers[topic_name] = rospy.Publisher(topic_name, PointCloud2, queue_size=10)
                self.clustered_point_msg[topic_name] = cluster_pc2_msg

            

            # Publish the cluster to its respective topic
            self.clustered_point_publishers[topic_name].publish(self.clustered_point_msg[topic_name])
            rospy.loginfo(f"Published cluster {cluster_id} to topic {topic_name} with {len(cluster_points)} points.")
        
        # publish number of clusters detected
        num_clusters_msg = Int32()
        num_clusters_msg.data = len(self.clusters)
        self.num_clusters_publisher.publish(num_clusters_msg)

        self.clustering_done = True  # Set the flag to indicate clustering is done

        return TriggerResponse(success=True, message="Clustering completed.")

    def publish_clusters(self, event=None):
        """Republish the latest clusters continuously."""
        for topic_name in self.clustered_point_publishers:
            self.clustered_point_publishers[topic_name].publish(self.clustered_point_msg[topic_name])


    def _find_optimal_eps(self, data, target_clusters=4):
        """Determine optimal eps for DBSCAN using a k-distance plot approach"""
        min_samples = 10
        nearest_neighbors = NearestNeighbors(n_neighbors=min_samples)
        neighbors = nearest_neighbors.fit(data)
        distances, _ = neighbors.kneighbors(data)
        distances = np.sort(distances[:, min_samples - 1])  # k-th nearest distances

        # Find the "elbow" point in the sorted k-distances
        diff = np.diff(distances)
        diff2 = np.diff(diff)
        elbow_point = np.argmax(diff2) + 2  # Adjust index to correct location
        optimal_eps = distances[elbow_point]

        # Apply a multiplier to increase `eps` if necessary
        optimal_eps *= 2.5

        rospy.loginfo(f"Optimal eps for DBSCAN based on k-distance elbow: {optimal_eps}")
        return optimal_eps

    def _parse_point_cloud(self, msg):
        """Parse PointCloud2 message into a list of 3D points"""
        pc_data = []

        # Calculate the number of bytes per point based on the field definitions
        point_step = msg.point_step
        data = np.frombuffer(msg.data, dtype=np.uint8)

        # Extract the x, y, z coordinates from the raw binary data
        for i in range(0, len(data), point_step):
            x = struct.unpack_from('f', data, i)[0]
            y = struct.unpack_from('f', data, i + 4)[0]
            z = struct.unpack_from('f', data, i + 8)[0]
            pc_data.append((x, y, z))

        return np.array(pc_data)

    def create_pointcloud2(self, points):
        """Create a PointCloud2 message from numpy array of points."""
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world_ned"  # Ensure the frame_id matches your setup in RViz

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Create PointCloud2 message
        pc2_msg = PointCloud2()
        pc2_msg.header = header
        pc2_msg.height = 1
        pc2_msg.width = len(points)
        pc2_msg.fields = fields
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 12  # 3 floats per point (x, y, z)
        pc2_msg.row_step = pc2_msg.point_step * len(points)
        pc2_msg.data = np.asarray(points, dtype=np.float32).tobytes()
        pc2_msg.is_dense = True

        return pc2_msg

def main():
    PointCloudCluster()
    rospy.spin()

if __name__ == '__main__':
    main()

