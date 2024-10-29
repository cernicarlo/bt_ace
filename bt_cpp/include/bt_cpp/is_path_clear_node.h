#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include <geometry_msgs/PointStamped.h>
#include "bt_cpp/utils.h"
#include <ros/ros.h>
#include <iostream>
#include <sstream>


#include "iauv_motion_planner/GetPath.h"
#include "iauv_motion_planner/GetPathRequest.h"
#include "iauv_motion_planner/PlannerParam.h"

#include "bt_cpp/utils.h"



#include <actionlib/client/simple_action_client.h>

#include "std_msgs/String.h"
#include "girona_utils/PursuitAction.h"

namespace IauvGirona1000Survey {


class isPathClear : public BT::CoroActionNode
{
public:
    isPathClear(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfig& config)
        : CoroActionNode(name, config), 
       nh_(nh),  
       is_object_detected_(false), 
       survey_type_(SCAN)
    {
        
        object_pose_sub_ = nh_.subscribe("/object_pose", 10, &isPathClear::objectPoseCallback, this);
    }
    // BT::NodeStatus onStart();
    // BT::NodeStatus onRunning();

    static BT::PortsList providedPorts() {
        return {}; // No ports
    }
    

private:
    // Tick function that checks the condition
    BT::NodeStatus tick() override final;

    // Callback function for the subscriber
    void objectPoseCallback(const geometry_msgs::PointStamped::ConstPtr& msg);

    // Helper function to manage log output frequency
    void printIfFromLastPrintHavePassedSomeSeconds(const std::string& msg, double seconds);

    ros::Subscriber object_pose_sub_;
    bool is_object_detected_;
    SurveyType survey_type_;
    std::chrono::steady_clock::time_point last_print_time_;
    ros::NodeHandle nh_;
};

}