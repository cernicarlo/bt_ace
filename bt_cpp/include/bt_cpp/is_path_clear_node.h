#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include <geometry_msgs/PointStamped.h>
#include "bt_cpp/utils.h"
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <chrono>


#include "iauv_motion_planner/GetPath.h"
#include "iauv_motion_planner/GetPathRequest.h"
#include "iauv_motion_planner/PlannerParam.h"

#include "bt_cpp/utils.h"



#include <actionlib/client/simple_action_client.h>

#include "std_msgs/String.h"
#include "girona_utils/PursuitAction.h"


// TODO: import it in the relative header/cpp file
class isPathClear : public BT::ConditionNode
{
private:
  std::string port_name_;
  ros::Subscriber object_pose_sub_;
  bool is_object_detected_;
  IauvGirona1000Survey::SurveyType survey_type_;
  std::chrono::steady_clock::time_point last_print_time_;
  ros::NodeHandle nh_;
  // TODO: these work only for this case, otherwise, we need a vector of points and some tolerance
  double prev_x_ , prev_y_, prev_z_;
  std::string prev_printed_msg_;

public:
  isPathClear(const std::string& name, const BT::NodeConfig& config,
                  std::string port_name, ros::NodeHandle nh)
    : BT::ConditionNode(name, config), port_name_(port_name), is_object_detected_(false), nh_(nh),
    prev_x_(NAN), prev_y_(NAN), prev_z_(NAN), prev_printed_msg_("")
  {
    object_pose_sub_ = nh_.subscribe("/object_pose", 10, &isPathClear::objectPoseCallback, this);
  }
  
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("survey_type") };
  };

  // Callback function for the subscriber
  void objectPoseCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
  
  BT::NodeStatus tick() override;
  
};
