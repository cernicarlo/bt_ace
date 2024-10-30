#!/usr/bin/env python


import rospy
from sensor_msgs.msg import PointCloud2
from pcl_geometric_primitives_detector.srv import Cluster, ClusterResponse
from pcl_geometric_primitives_detector.msg import ClusterObjInfo
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score
import matplotlib.pyplot as plt
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

octomap_data = None

def octomap_callback(msg):
    global octomap_data
    points = np.array(list(pc2.read_points(msg, skip_nans=True)))
    octomap_data = points
    rospy.loginfo("Received octomap data.")

def handle_cluster(req):
    global octomap_data
    if octomap_data is None:
        return ClusterResponse(success=False, message="Octomap data not available.")
    
    points = octomap_data
    optimal_clusters = find_optimal_clusters(points)
    kmeans = KMeans(n_clusters=optimal_clusters)
    kmeans.fit(points)
    cluster_labels = kmeans.labels_
    
    object_info_list = []  # List to store the ClusterObjInfo for each cluster

    for i in range(optimal_clusters):
        cluster_points = points[cluster_labels == i]
        
        # Calculate the centroid of the cluster
        centroid = np.mean(cluster_points, axis=0)

        # Create surface point
        surface_point = calculate_surface_point(cluster_points)

        # Create PointCloud2 message for the clustered points
        cluster_pc2_msg = create_pointcloud2(cluster_points)

        # Create and fill ClusterObjInfo for the current cluster
        cluster_obj_info = ClusterObjInfo()
        cluster_obj_info.centroid.x = centroid[0]
        cluster_obj_info.centroid.y = centroid[1]
        cluster_obj_info.centroid.z = centroid[2]
        cluster_obj_info.surface_point.x = surface_point[0]
        cluster_obj_info.surface_point.y = surface_point[1]
        cluster_obj_info.surface_point.z = surface_point[2]
        cluster_obj_info.clustered_pointcloud = cluster_pc2_msg

        # Append this object info to the list
        object_info_list.append(cluster_obj_info)

        # Publish TF for centroid and surface point
        tf_broadcast(centroid, "cluster_centroid_" + str(i))
        tf_broadcast(surface_point, "cluster_surface_point_" + str(i))

    # Publish object information (list of ClusterObjInfo)
    publish_object_info(object_info_list)

    # Returning ClusterResponse
    cluster_response = ClusterResponse()
    cluster_response.success = True
    cluster_response.message = "Clustering completed successfully."
    return cluster_response

def create_pointcloud2(points):
    """
    Create a PointCloud2 message from numpy array of points.
    Each point in 'points' should have (x, y, z) coordinates.
    """
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "world_ned"

    pc2_msg = pc2.create_cloud(header, fields, points)
    return pc2_msg

def find_optimal_clusters(data):
    silhouette_scores = []
    max_clusters = 5 

    for i in range(2, max_clusters + 1):
        kmeans = KMeans(n_clusters=i, random_state=42)
        cluster_labels = kmeans.fit_predict(data)
        silhouette_avg = silhouette_score(data, cluster_labels)
        silhouette_scores.append(silhouette_avg)

    plt.plot(range(2, max_clusters + 1), silhouette_scores, marker='o')
    plt.xlabel('Number of clusters')
    plt.ylabel('Silhouette Score')
    plt.title('Silhouette Score for Optimal Number of Clusters')
    plt.show()

    optimal_clusters = np.argmax(silhouette_scores) + 2  # Adding 2 because the range starts from 2
    rospy.loginfo(optimal_clusters)
    return optimal_clusters

def calculate_surface_point(cluster_points):
    centroid = np.mean(cluster_points, axis=0)
    surface_point = np.max(cluster_points[np.logical_and(cluster_points[:, :2] == centroid[:2], cluster_points[:, 2] == np.max(cluster_points[:, 2]))], axis=0)
    return surface_point

def tf_broadcast(point, frame_id):
    # Create a TF broadcaster
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Create a TF message
    tf_msg = geometry_msgs.msg.TransformStamped()
    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.header.frame_id = "world_ned"
    tf_msg.child_frame_id = frame_id
    tf_msg.transform.translation.x = point[0]
    tf_msg.transform.translation.y = point[1]
    tf_msg.transform.translation.z = point[2]
    tf_msg.transform.rotation.w = 1.0  # No rotation

    # Broadcast the TF message
    tf_broadcaster.sendTransform(tf_msg)

def publish_object_info(object_info_list):
    object_info_pub = rospy.Publisher('clustered_point_cloud', ClusterObjInfo, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    for object_info in object_info_list:
        object_info_pub.publish(object_info)
        rate.sleep()

def clustering_server():
    rospy.init_node('ACE_clustering_server') 
    rospy.Subscriber('/voxel_grid/output', PointCloud2, octomap_callback)
    s = rospy.Service('cluster', Cluster, handle_cluster)
    print("Ready to cluster.")
    rospy.spin()

if __name__ == '__main__':
    clustering_server()

