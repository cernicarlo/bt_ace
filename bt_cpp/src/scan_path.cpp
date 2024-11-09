#include "behaviortree_cpp/bt_factory.h"
#include "bt_cpp/follow_path_node.h"
#include "bt_cpp/path_request_node.h"
#include "bt_cpp/is_path_clear_node.h"
#include "bt_cpp/log_nodes.h"
#include <fstream>
#include <iostream>
#include <cmath>

using namespace BT;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bt_cpp_node");

  std::string abs_repo_path = "/home/ros/catkin_ws/src/";
  std::string relative_xml_fold_path = "bt_ace/bt_cpp/bt_xml/";
  std::string xml_tree = "iauv_girona1000_survey_scan.xml";
  // BehaviorTreeFactory factory;

  std::unique_ptr<BT::BehaviorTreeFactory> factory;
  std::string path;
  ros::NodeHandle nh;
  // path = "/home/ros/catkin_ws/src/bt_ace/bt_cpp/bt_xml/iauv_girona1000_survey_circular_true.xml";
  path = abs_repo_path + relative_xml_fold_path + xml_tree;
  
  factory = std::make_unique<BT::BehaviorTreeFactory>();
  
  // iauv girona1000 survey
  registerCustomNode<IauvGirona1000Survey::PathRequest>(
      *factory, "PathRequest", nh);


  // registerCustomNode<IauvGirona1000Survey::isPathClear>(
  //     *factory, "isPathClear", nh);
  
  factory->registerNodeType<isPathClear>("isPathClear", "is_path_clear", nh);
  // factory->registerNodeType<IauvGirona1000Survey::isPathClear>("isPathClear", "is_path_clear");
  // registerSimpleCondition(
  //         "isPathClear",
  //         std::bind(IauvGirona1000Survey::FollowPath::isPathClear));

  PortsList say_something_ports = { InputPort<std::string>("message") };
  factory->registerSimpleAction("LogSequence", LogSequence, say_something_ports);
  factory->registerSimpleAction("LogFallback", LogFallback, say_something_ports);

  // factory->registerNodeType<DummyNodes::SaySomething>("SaySomething");
  registerCustomNode<IauvGirona1000Survey::FollowPath>(
      *factory, "FollowPath", nh);
   
  
  // factory.registerNodeType<SaySomething>("SaySomething");
  // factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");

  // Parse the XML file and create a tree_ from it
  factory->registerBehaviorTreeFromFile(path);
  std::unique_ptr<BT::Tree> tree;
  std::string bt_id = "main_bt";
  tree = std::make_unique<BT::Tree>(
       factory->createTree(bt_id));
  
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  while (status != BT::NodeStatus::SUCCESS &&
          status != BT::NodeStatus::FAILURE && ros::ok()) {
    status = tree->tickOnce();      // Tick the tree once
    ros::spinOnce();       // Process ROS callbacks
    // loop_rate.sleep();     // Sleep to maintain loop rate
  }

  return 0;
}
