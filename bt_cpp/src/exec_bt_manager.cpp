#include "behaviortree_cpp/bt_factory.h"
#include "bt_cpp/bt_manager.h"
#include "bt_cpp/follow_path_node.h"
#include "bt_cpp/path_request_node.h"
#include "bt_cpp/is_path_clear_node.h"
#include "bt_cpp/log_nodes.h"
#include "bt_cpp/utils.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <std_srvs/Trigger.h>

#include <ros/package.h>

using namespace BT;

int main(int argc, char** argv)
{
/*
  variables initialization
*/ 
  // ROS init
  ros::init(argc, argv, "bt_cpp_node");
  ros::NodeHandle nh;

  // path init
  std::string xml_path = ros::package::getPath("bt_cpp") + "/bt_xml/";

  // Factory init
  std::unique_ptr<BT::BehaviorTreeFactory> factory;
  factory = std::make_unique<BT::BehaviorTreeFactory>();
  // initialize possible BT nodes
  registerCustomNode<IauvGirona1000Survey::PathRequest>(
      *factory, "PathRequest", nh);
  registerCustomNode<IauvGirona1000Survey::FollowPath>(
      *factory, "FollowPath", nh);
  factory->registerNodeType<isPathClear>("isPathClear", "is_path_clear", nh);
  PortsList say_something_ports = { InputPort<std::string>("message") };
  factory->registerSimpleAction("LogSequence", LogSequence, say_something_ports);
  factory->registerSimpleAction("LogFallback", LogFallback, say_something_ports);

  // BTManager
  BTManager bt_manager(xml_path, nh, *factory);


/*
  Mission
*/ 

/// Validate Mission
  // Create a service client for the /validate_mission service
  ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/validate_mission");

  // Create the service request and response objects
  std_srvs::Trigger srv;

  // Call the service and check if it was successful
  if (client.call(srv)) {
      if (srv.response.success) {
          ROS_INFO("Mission validation successful: %s", srv.response.message.c_str());
      } else {
          ROS_ERROR("Mission validation failed: %s", srv.response.message.c_str());
          return 1;
      }
  } else {
      ROS_ERROR("Failed to call service /validate_mission");
      return 1;
  }


/// Create stack for mission
  // Get the file path from the /mission_file parameter
  std::string mission_file_path;
  if (!nh.getParam("/mission_file", mission_file_path)) {
      ROS_ERROR("Failed to get /mission_file parameter");
      return 1;
  }

  // Open the mission file
  std::ifstream mission_file(mission_file_path);
  if (!mission_file.is_open()) {
      ROS_ERROR("Failed to open mission file: %s", mission_file_path.c_str());
      return 1;
  }

  // Process each line of the file
  std::vector<std::string> action_targets;
  std::string line;
  while (std::getline(mission_file, line)) {
      std::istringstream iss(line);
      std::string subject, action, target;

      // Parse the line into subject, action, and target
      if (!(iss >> subject >> action >> target)) {
          ROS_WARN("Skipping malformed line: %s", line.c_str());
          continue;
      }

      // If the subject is "AUV", create and store the action_target string
      if (subject == "AUV") {
          std::string action_target = action + "_" + target;
          action_targets.push_back(action_target);
          ROS_INFO("Created action_target: %s", action_target.c_str());
      }
  }

  mission_file.close();


  // Start mission taking BT from mission file
  bt_manager.populateStack(action_targets);


/// Execute stack mission
  bt_manager.executeStack();
  std::cout << "First stack executed" << std::endl;

  // go to Luma
  bt_manager.populateStack({"goto_luma"});
  bt_manager.executeStack();
  std::cout << "Go to LUMA executed" << std::endl;

  // repopulate the stack with object info
  bt_manager.populateStack({"observe_cube"});
  bt_manager.executeStack();
  std::cout << "Last stack executed" << std::endl;

  return 0;
}

