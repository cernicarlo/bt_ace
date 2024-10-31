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

pcd_data = None

def pointcloud_callback(msg):
    global pcd_data
    points = np.array(list(pc2.read_points(msg, skip_nans=True)))
    pcd_data = points
    rospy.loginfo("Received PointCloud data.")

def handle_cluster(req):
    global pcd_data
    cluster_response = ClusterResponse()


    if pcd_data is None:
        cluster_response.success = False
        cluster_response.message = "PointCloud data not available."
        cluster_response.points = []
        return cluster_response
    
    points = pcd_data
    optimal_clusters = find_optimal_clusters(points)
    kmeans = KMeans(n_clusters=optimal_clusters)
    kmeans.fit(points)
    cluster_labels = kmeans.labels_
    
    object_info_list = []  # List to store the ClusterObjInfo for each cluster
    centroids = []

    for i in range(optimal_clusters):
        cluster_points = points[cluster_labels == i]
        
        # Calculate the centroid of the cluster
        centroid = np.mean(cluster_points, axis=0)
        centroids.extend(centroid.tolist())

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
    
    cluster_response.success = True
    cluster_response.message = "Clustering completed successfully."
    cluster_response.points = centroids

    print("Type of cluster_response.points:", type(cluster_response.points))
    print("Contents of cluster_response.points:", cluster_response.points)
    return cluster_response.success, cluster_response.message, cluster_response.points

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

    fig, ax = plt.subplots()
    ax.plot(range(2, max_clusters + 1), silhouette_scores, marker='o')
    ax.set_xlabel('Number of clusters')
    ax.set_ylabel('Silhouette Score')
    ax.set_title('Silhouette Score for Optimal Number of Clusters')
    fig.savefig('silhouette_scores.png')
    plt.close(fig)

    optimal_clusters = np.argmax(silhouette_scores) + 2  # Adding 2 because the range starts from 2
    rospy.loginfo(optimal_clusters)
    return optimal_clusters

def calculate_surface_point(cluster_points):
    centroid = np.mean(cluster_points, axis=0)
    # Condition with np.all and np.isclose for better precision handling
    condition = np.logical_and(
        np.all(np.isclose(cluster_points[:, :2], centroid[:2]), axis=1),
        cluster_points[:, 2] == np.max(cluster_points[:, 2])
    )
    if np.any(condition):
        surface_point = np.max(cluster_points[condition], axis=0)
    else:
        surface_point = centroid  # Fallback if no points match
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
    object_info_pub = rospy.Publisher('/clustered_point_cloud', ClusterObjInfo, queue_size=10)
    object_cluster_pub = rospy.Publisher('/clustered_point_cloud_visual', PointCloud2, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    print("going to publish object_info")

    for object_info in object_info_list:
        object_info_pub.publish(object_info)
        object_cluster_pub.publish(object_info.clustered_pointcloud)
        print("published object_info")
        rate.sleep()

def clustering_server():
    rospy.init_node('ACE_clustering_server') 
    rospy.Subscriber('/luma_station/compressed_cloud', PointCloud2, pointcloud_callback)
    s = rospy.Service('cluster', Cluster, handle_cluster)
    print("Ready to cluster.")
    rospy.spin()

if __name__ == '__main__':
    clustering_server()

