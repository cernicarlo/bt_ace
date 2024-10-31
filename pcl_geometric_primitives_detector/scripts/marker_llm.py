#!/usr/bin/env python

import rospy
import numpy as np
from pcl_geometric_primitives_detector.msg import LabeledObjInfo
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from std_msgs.msg import Header
import os
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from groq import Groq

# Global variables
server = None
menu_handlers = {}
h_first_entry = 0
h_mode_last = 0


client = Groq(

    api_key='',

)



def pointcloud_to_list(cloud_msg):
    """
    Convert a sensor_msgs/PointCloud2 message to a list of points.
    Each point is represented as a tuple (x, y, z).
    """
    points = []
    
    # Extract points from the PointCloud2 message
    for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append((point[0], point[1], point[2]))  # Append each point (x, y, z) as a tuple
    
    return points

def label_obj_callback(msg):
    global server

    # Extract object information from the message
    cluster_label = msg.label
    cluster_point_cloud = msg.point_cloud  # Assuming the message contains a PointCloud2
    points = pointcloud_to_list(cluster_point_cloud)  # Convert PointCloud2 to a NumPy array

    # Calculate the centroid of the cluster's point cloud
    centroid = calculate_centroid(points)

    # Marker name
    marker_name = f'marker_{cluster_label}'
    
    # Create or update the marker
    if server.get(marker_name):
        rospy.loginfo(f'Updating marker for cluster {cluster_label}')
    else:
        rospy.loginfo(f'Creating new marker for cluster {cluster_label}')
        makeMenuMarker(marker_name, points)

        # Initialize a unique menu for the marker
        initMenu(menu_handlers[marker_name], cluster_label)
    
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

def enableCb(feedback):
    """handle = feedback.menu_entry_id
    state = menu_handler.getCheckState(handle)

    if state == MenuHandler.CHECKED:
        menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
        node.get_logger().info('Hiding first menu entry')
        menu_handler.setVisible(h_first_entry, False)
    else:
        menu_handler.setCheckState(handle, MenuHandler.CHECKED)
        node.get_logger().info('Showing first menu entry')
        menu_handler.setVisible(h_first_entry, True)

    menu_handler.reApply(server)
    server.applyChanges()
    """

def initMenu(menu, label):
    # Add the last entry with two sentences
    chat_completion = client.chat.completions.create(

    messages=[

        {

            "role": "user",

            "content": "what's a "+label+"?",

        }

    ],

    model="llama3-8b-8192",

    )

    # Sample text from chat completion
    ai_generated_text = chat_completion.choices[0].message.content

    # Split the text into words
    words = ai_generated_text.split()

    # Create entries for the menu handler
    for i in range(0, len(words), 5):
        # Join the next 10 words and create a menu entry
        entry = ' '.join(words[i:i + 5]).replace("...**Cut**...","").replace("**","")
        # Insert each entry into the menu handler
        last_entry_handle = menu.insert(entry, callback=enableCb)

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

    menu_handlers = {}  # Initialize menu handlers
    rospy.Subscriber('/labeled_cloud', LabeledObjInfo, label_obj_callback, queue_size=10)

    rospy.spin()
    server.shutdown()

if __name__ == '__main__':
    main()

