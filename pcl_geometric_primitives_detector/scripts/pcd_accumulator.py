#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
class PointCloudAccumulator:

    def __init__(self, topic_name="/girona1000/depth_cam/pointcloud"):
        # Initialize ROS node
        rospy.init_node('pointcloud_accumulator', anonymous=True)
        # Set up the subscriber and publisher
        self.subscriber = rospy.Subscriber(topic_name, PointCloud2, self.pointcloud_callback)
        self.publisher = rospy.Publisher("/girona1000/accumulated_pointcloud", PointCloud2, queue_size=1)
        # Initialize an empty point cloud array
        self.accumulated_points = np.empty((0, 3), dtype=np.float32)

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 message to numpy array
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        # Append the new points to the accumulated points
        self.accumulated_points = np.vstack((self.accumulated_points, points))
        # Remove duplicate points if needed
        self.accumulated_points = np.unique(self.accumulated_points, axis=0)
        # Convert the accumulated points back to PointCloud2 and publish
        self.publish_accumulated_pointcloud()

    def publish_accumulated_pointcloud(self):
        # Create a new PointCloud2 message from accumulated points
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "girona1000/depth_cam_link"  # Change frame ID as necessary
        accumulated_pc2 = pc2.create_cloud_xyz32(header, self.accumulated_points.tolist())
        self.publisher.publish(accumulated_pc2)

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == "__main__":
    # Initialize the accumulator node with the specified topic
    topic_name = "/girona1000/depth_cam/pointcloud"  # Replace with the input topic name if different
    accumulator = PointCloudAccumulator(topic_name)
    accumulator.run()