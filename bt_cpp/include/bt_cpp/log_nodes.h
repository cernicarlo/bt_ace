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

BT::NodeStatus LogSequence(BT::TreeNode& self)
{
  auto msg = self.getInput<std::string>("message");
  if(!msg)
  {
    throw BT::RuntimeError("missing required input [message]: ", msg.error());
  }

  std::cout << "Robot says: " << msg.value() << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LogFallback(BT::TreeNode& self)
{
  auto msg = self.getInput<std::string>("message");
  if(!msg)
  {
    throw BT::RuntimeError("missing required input [message]: ", msg.error());
  }

  std::cout << "Robot says: " << msg.value() << std::endl;
  return BT::NodeStatus::SUCCESS;
}