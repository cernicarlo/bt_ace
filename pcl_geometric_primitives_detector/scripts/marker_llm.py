#!/usr/bin/env python

import rospy
import numpy as np
from pcl_geometric_primitives_detector.msg import ClusterObjInfo
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from std_msgs.msg import Header

# Global variables
server = None
menu_handlers = {}
h_first_entry = 0
h_mode_last = 0

def clustered_obj_callback(msg):
    global server

    # Extract object information from the message
    cluster_id = msg.cluster_id
    cluster_point_cloud = msg.point_cloud  # Assuming the message contains a PointCloud2
    points = convert_pointcloud_to_array(cluster_point_cloud)  # Convert PointCloud2 to a NumPy array

    # Calculate the centroid of the cluster's point cloud
    centroid = calculate_centroid(points)

    # Marker name
    marker_name = f'marker_{cluster_id}'
    
    # Create or update the marker
    if server.get(marker_name):
        rospy.loginfo(f'Updating marker for cluster {cluster_id}')
    else:
        rospy.loginfo(f'Creating new marker for cluster {cluster_id}')
        makeMenuMarker(marker_name, points)

        # Initialize a unique menu for the marker
        initMenu(menu_handlers[marker_name])
    
    # Apply the marker-specific menu handler
    menu_handlers[marker_name].apply(server, marker_name)
    server.applyChanges()

def makeBox(msg):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.3
    marker.scale.y = msg.scale * 0.3
    marker.scale.z = msg.scale * 0.3
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    return marker

def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control

def makeEmptyMarker(centroid):
    int_marker = InteractiveMarker()
    int_marker.header = Header(frame_id='map')
    
    # Place marker at the centroid of the point cloud
    int_marker.pose.position.x = centroid[0]
    int_marker.pose.position.y = centroid[1]
    int_marker.pose.position.z = centroid[2]

    int_marker.scale = 0.5  # Make the marker smaller
    return int_marker

def makeMenuMarker(name, points):
    centroid = calculate_centroid(points)
    int_marker = makeEmptyMarker(centroid)
    int_marker.name = name

    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    control.markers.append(makeBox(int_marker))
    int_marker.controls.append(control)

    server.insert(int_marker)

def initMenu(menu):
    # Add entries to the menu handler
    entry_handle = menu.insert("Sample Entry", callback=enableCb)

def calculate_centroid(points):
    if len(points) == 0:
        rospy.logerr("Error: No points available to calculate the centroid.")
        return None
    
    # Proceed if points are available
    centroid = np.mean(points, axis=0)
    return centroid

def main():
    global server
    rospy.init_node('menu', anonymous=True)
    server = InteractiveMarkerServer("menu")

    # Initialize menu handlers
    menu_handlers = {}

    # Set up the subscriber in ROS 1 style
    rospy.Subscriber('clustered_obj_info', ClusterObjInfo, clustered_obj_callback)

    rospy.spin()
    server.clear()
    server.applyChanges()


if __name__ == '__main__':
    main()

