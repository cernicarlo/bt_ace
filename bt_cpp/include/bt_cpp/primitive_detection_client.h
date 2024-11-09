#ifndef PRIMITIVE_DETECTION_CLIENT_H
#define PRIMITIVE_DETECTION_CLIENT_H

#include <ros/ros.h>
#include <bt_cpp/LabeledObjInfo.h>
#include <bt_cpp/GetActions.h>
#include <geometry_msgs/Point.h>
#include "bt_cpp/Cluster.h"
#include <string>
#include <vector>
#include <unordered_map>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

struct ObjectInfo {
    geometry_msgs::Point surface_point;
    std::vector<std::string> actions;
};

class PrimitiveDetectionClient {
public:
    PrimitiveDetectionClient(ros::NodeHandle& nh);
    void labeledObjInfoCallback(const bt_cpp::LabeledObjInfo::ConstPtr& msg);
    std::unordered_map<std::string, ObjectInfo> getAllDetectedObjects() const;
    void numClustersCallback(const std_msgs::Int32::ConstPtr& msg);
    bool isProcessingComplete() const;
    // methods
    void clusterPointcloud();
    int getNumberClouds();
    bool isCloudDetected();
    void lumaDetectedCallback(const std_msgs::Bool::ConstPtr& msg);
    bool is_luma_detected_;


private:
    ros::NodeHandle nh_;
    ros::Subscriber labeled_obj_sub_;
    ros::Subscriber num_clusters_sub_;
    ros::Subscriber luma_detected_sub_;
    ros::ServiceClient get_actions_client_;
    std::unordered_map<std::string, ObjectInfo> labeled_objects_;
    int num_clusters_used_;
    int processed_clusters_;
    // pointcloud processor variables
    bool is_cloud_detected_;
    bool is_clustering_being_called_;
    ros::ServiceClient cluster_client_;
    bt_cpp::Cluster cluster_srv_;
    int num_clouds_;
    int max_clouds_;
    ros::Time last_cluster_call_time_;
    double frequency_cluster_call_;  
};

#endif // PRIMITIVE_DETECTION_CLIENT_H
