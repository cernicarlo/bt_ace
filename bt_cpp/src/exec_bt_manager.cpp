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

  // Start mission taking BT from mission file
  bt_manager.populateStack({"scan", "observe_sphere"});
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

