#!/usr/bin/env python

import rospy
from pcl_geometric_primitives_detector.msg import ClusterObjInfo, LabeledObjInfo
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2

class PrimitiveDetectionNode:
    def __init__(self):
        rospy.init_node('primitive_detection_node', anonymous=True)
        
        # Subscriber
        self.subscription = rospy.Subscriber(
            "/vlc_computer/compressed_cloud_0", PointCloud2, self.point_cloud_callback
        )
        
        # Publisher
        self.publisher = rospy.Publisher(
            "/labeled_obj_info_mock", LabeledObjInfo, queue_size=10
        )
        
        # Counter to keep track of the message sequence
        self.publish_count = 0

    def point_cloud_callback(self, msg):
        # Create and populate LabeledObjInfo message
        labeled_msg = LabeledObjInfo()
        
        # Cycle through different configurations based on the publish count
        if self.publish_count == 0:
            labeled_msg.centroid = Point(x=3.0, y=5.0, z=5.5)
            labeled_msg.surface_point = Point(x=3.0, y=5.0, z=6.5)
            labeled_msg.label = "box"
        elif self.publish_count == 1:
            labeled_msg.centroid = Point(x=0.0, y=5.0, z=5.5)
            labeled_msg.surface_point = Point(x=0.0, y=5.0, z=7.5)
            labeled_msg.label = "sphere"
        elif self.publish_count == 2:
            labeled_msg.centroid = Point(x=0.0, y=4.0, z=5.5)
            labeled_msg.surface_point = Point(x=0.0, y=4.0, z=5.5)
            labeled_msg.label = "cone"
        elif self.publish_count == 3:
            labeled_msg.centroid = Point(x=3.0, y=0.0, z=5.5)
            labeled_msg.surface_point = Point(x=3.0, y=0.0, z=5.5)
            labeled_msg.label = "box"
        else:
            labeled_msg.centroid = Point(x=3.0, y=0.0, z=5.5)
            labeled_msg.surface_point = Point(x=3.0, y=0.0, z=5.5)
            labeled_msg.label = "box"
            print(f"published redundant object (probably, luma station), checking little box")

        
        # Copy the clustered_pointcloud from the received message
        labeled_msg.clustered_pointcloud = msg
        
        # Publish the labeled message
        self.publisher.publish(labeled_msg)
        print(f"published pc_cb - {labeled_msg.label}")

        # Increment the publish count and reset after 3 to repeat the cycle
        self.publish_count = (self.publish_count + 1) % 3

if __name__ == '__main__':
    try:
        PrimitiveDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
