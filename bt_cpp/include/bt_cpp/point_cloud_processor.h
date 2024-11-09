#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "bt_cpp/Cluster.h"

class PointCloudProcessor {
public:
    PointCloudProcessor(ros::NodeHandle& nh) : is_cloud_detected_(false)
        {
        // Initialize the subscriber
        pointcloud_sub_ = nh.subscribe("/luma_station/compressed_cloud", 10, &PointCloudProcessor::pointcloudCallback, this);

        // Initialize the service client for the /cluster service
        cluster_client_ = nh.serviceClient<bt_cpp::Cluster>("/cluster");
    }

    int getNumberClouds(){
        ROS_INFO("getting num_clouds detected: %i", num_clouds_);

        return num_clouds_;
    }

    bool isCloudDetected(){
        return is_cloud_detected_;
    }

private:
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        ROS_INFO("Received point cloud data");

        // Create a service request and response
        bt_cpp::Cluster srv;
        // Call the service and check if it was successful
        if (cluster_client_.call(srv) && !is_cloud_detected_) {
            if (srv.response.success) {
                is_cloud_detected_ = true;
                ROS_INFO("Cluster service call successful: %s", srv.response.message.c_str());
                ROS_INFO("Points: ");
                for (const auto& point : srv.response.points) {
                    ROS_INFO("%f", point);
                }
                num_clouds_ = srv.response.points.size()/3;
                ROS_INFO("num_clouds detected: %i", num_clouds_);
            } else {
                ROS_WARN("Cluster service call failed: %s", srv.response.message.c_str());
            }
        } else {
            ROS_ERROR("Failed to call service /cluster");
        }
    }

    ros::Subscriber pointcloud_sub_;
    ros::ServiceClient cluster_client_;
    int num_clouds_;
    bool is_cloud_detected_ = false;
    bt_cpp::Cluster cluster_srv_;
};