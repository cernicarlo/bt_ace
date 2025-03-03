#include "bt_cpp/primitive_detection_client.h"
#include <ros/ros.h>
#include "bt_cpp/LabeledObjInfo.h"
#include "bt_cpp/GetActions.h"
#include <string>
#include <vector>
#include <iostream>
#include <std_srvs/Trigger.h> 

PrimitiveDetectionClient::PrimitiveDetectionClient(ros::NodeHandle& nh) 
    : nh_(nh), processed_clusters_(0),
        is_cloud_detected_(false), 
        last_cluster_call_time_(ros::Time::now()), 
        frequency_cluster_call_(1.0), 
        is_luma_detected_(false),
        is_clustering_being_called_(false)  {
    
    // Set up a subscriber to continuously update num_clusters_
    num_clusters_sub_ = nh_.subscribe("/num_clusters", 10, &PrimitiveDetectionClient::numClustersCallback, this);

    // Initialize the subscriber and service client
    labeled_obj_sub_ = nh_.subscribe("/labeled_obj_info_mock", 10, &PrimitiveDetectionClient::labeledObjInfoCallback, this);

    // Set up a subscriber to /is_luma_detected
    luma_detected_sub_ = nh_.subscribe("/is_luma_detected", 10, &PrimitiveDetectionClient::lumaDetectedCallback, this);

    cluster_client_ = nh.serviceClient<std_srvs::Trigger>("/clustering");
    // TODO: change this as soon as the get_actions is fixed
    get_actions_client_ = nh_.serviceClient<bt_cpp::GetActions>("/get_actions");
    
    
}

void PrimitiveDetectionClient::lumaDetectedCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        ROS_INFO("Luma detected.");
        clusterPointcloud();
        is_luma_detected_ = true;
    } else {
        is_luma_detected_ = false;
    }
}
// // Subscribe to /num_clusters and retrieve the last message published
//     auto last_msg = ros::topic::waitForMessage<std_msgs::Int32>("/num_clusters", nh_);
//     if (last_msg) {
//         num_clusters_ = last_msg->data;
//         ROS_INFO("Retrieved initial number of clusters: %d", num_clusters_);
//     }


void PrimitiveDetectionClient::numClustersCallback(const std_msgs::Int32::ConstPtr& msg) {
    // Update num_clusters_ with the latest data
    num_clouds_ = msg->data;
    max_clouds_ = num_clouds_ - 1;
    ROS_INFO("Updated num_clouds_: %d", num_clouds_);
}

int PrimitiveDetectionClient::getNumberClouds(){
    ROS_INFO("getting num_clouds detected: %i", num_clouds_);

    return num_clouds_;
}

bool PrimitiveDetectionClient::isCloudDetected(){
    return is_cloud_detected_;
}

void PrimitiveDetectionClient::clusterPointcloud() {
    if(is_clustering_being_called_){
        return;
    }
    ROS_INFO("Luma detected. Triggering clusterPointcloud.");
    last_cluster_call_time_ = ros::Time::now();
    
    // Create a service request and response
    std_srvs::Trigger srv;
    ROS_INFO("Going to call cluster service");
    // Call the service and check if it was successful
    if (cluster_client_.call(srv)) {
        if (srv.response.success) {
            is_cloud_detected_ = true;
            ROS_INFO("Cluster service call successful");
            is_clustering_being_called_ = true;
        } else {
            ROS_WARN("Cluster service call failed");
        }
    } else {
        ROS_ERROR("Failed to call service /cluster");
    }
    
}

void PrimitiveDetectionClient::labeledObjInfoCallback(const bt_cpp::LabeledObjInfo::ConstPtr& msg) {
    // Extract label, centroid, and surface_point
    std::string label = msg->label;
    geometry_msgs::Point surface_point = msg->surface_point;

    // Prepare the service request
    bt_cpp::GetActions srv;
    srv.request.label = label;

    // Call the service to get actions
    if (get_actions_client_.call(srv)) {
        // Store the surface_point and actions in the map
        ObjectInfo obj_info;
        obj_info.surface_point = surface_point;
        obj_info.actions = srv.response.actions;
        labeled_objects_[label] = obj_info;

        // Increment processed clusters count
        processed_clusters_++;

        // Output the information
        ROS_INFO("Processed label: %s", label.c_str());
    } else {
        ROS_ERROR("Failed to call service /get_actions_mock for label: %s", label.c_str());
    }
}

std::unordered_map<std::string, ObjectInfo> PrimitiveDetectionClient::getAllDetectedObjects() const {
    return labeled_objects_;
}

bool PrimitiveDetectionClient::isProcessingComplete() const {
    return (processed_clusters_ >= max_clouds_);
}