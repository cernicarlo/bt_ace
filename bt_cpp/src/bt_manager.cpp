#include "behaviortree_cpp/bt_factory.h"
#include "bt_cpp/follow_path_node.h"
#include "bt_cpp/path_request_node.h"
#include "bt_cpp/is_path_clear_node.h"
#include "bt_cpp/log_nodes.h"
#include <fstream>
#include <iostream>
#include <cmath>

#include <ros/package.h>

using namespace BT;

template <typename NodeType>
void registerCustomNode(BT::BehaviorTreeFactory& factory,
                        const std::string& registration_id,
                        ros::NodeHandle& nh) {
   // Use the registration_id directly as the type identifier
   BT::NodeBuilder builder = [&nh](
                                 const std::string& name,
                                 const BT::NodeConfig& config) {
      return std::make_unique<NodeType>(nh, name, config);
   };

   // Assuming you directly use the registration_id as the manifest type
   factory.registerBuilder<NodeType>(registration_id, builder);
}



void BtRun(BT::NodeStatus &status, std::unique_ptr<BT::Tree> &tree) {
   while (status != BT::NodeStatus::SUCCESS &&
          status != BT::NodeStatus::FAILURE &&
          ros::ok()) {
            status = tree->tickOnce();
            ros::spinOnce();
          }
}



int main(int argc, char** argv)
{
  /// variables initialization

  // ROS init
  ros::init(argc, argv, "bt_cpp_node");
  ros::NodeHandle nh;

  // path init
  // (TODO: make it dynamic) change this according to where you installed the package
  std::string abs_repo_path = ros::package::getPath("bt_cpp");
  std::string relative_xml_fold_path = "/bt_xml/";
  std::string extension = ".xml";
  std::string path;

  // BT init
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  std::unique_ptr<BT::BehaviorTreeFactory> factory;
  factory = std::make_unique<BT::BehaviorTreeFactory>();
  std::unique_ptr<BT::Tree> tree;
  std::string bt_id = "main_bt";
  // initialize possible BT nodes
  registerCustomNode<IauvGirona1000Survey::PathRequest>(
      *factory, "PathRequest", nh);
  registerCustomNode<IauvGirona1000Survey::FollowPath>(
      *factory, "FollowPath", nh);
  factory->registerNodeType<isPathClear>("isPathClear", "is_path_clear", nh);
  PortsList say_something_ports = { InputPort<std::string>("message") };
  factory->registerSimpleAction("LogSequence", LogSequence, say_something_ports);
  factory->registerSimpleAction("LogFallback", LogFallback, say_something_ports);

  
  // stack  
  std::deque<std::string> stack;
  std::string bt_exec;


  /// path population
  // TODO: function to populate stack
  bt_exec = "BT_1";
  stack.push_back(bt_exec);
  bt_exec = "BT_2";
  stack.push_back(bt_exec);
  bt_exec = "BT_3";
  stack.push_back(bt_exec);

  /// execute BT in stack to start mission
  while(!stack.empty()) {

    std::string xml_tree = stack.front();
    stack.pop_front();
    path = abs_repo_path + relative_xml_fold_path + xml_tree + extension;
    factory->registerBehaviorTreeFromFile(path);
    tree = std::make_unique<BT::Tree>(
            factory->createTree(bt_id));
    BtRun(status, tree);

    tree = std::make_unique<BT::Tree>(
                factory->createTree(bt_id));
    status = BT::NodeStatus::IDLE;
                    

  }
  std::cout<<"first stack executed" <<std::endl;
  
  /// go to check to LUMA
  std::string xml_tree = "bt_goto_luma";
  path = abs_repo_path + relative_xml_fold_path + xml_tree + extension;
  factory->registerBehaviorTreeFromFile(path);
  tree = std::make_unique<BT::Tree>(
            factory->createTree(bt_id));
    BtRun(status, tree);

    tree = std::make_unique<BT::Tree>(
                factory->createTree(bt_id));
    status = BT::NodeStatus::IDLE;
  std::cout<<"goto luma executed" <<std::endl;

  /// according to LUMA reply, populate new stack
  bt_exec = "BT_4";
  stack.push_back(bt_exec);
  bt_exec = "BT_5";
  stack.push_back(bt_exec);
  bt_exec = "BT_6";
  stack.push_back(bt_exec);

  /// execute BT in stack to finish the mission with the tasks assigned by luma
  while(!stack.empty()) {
      std::string xml_tree = stack.front();
      stack.pop_front();
      path = abs_repo_path + relative_xml_fold_path + xml_tree + extension;
      factory->registerBehaviorTreeFromFile(path);
      tree = std::make_unique<BT::Tree>(
            factory->createTree(bt_id));
        BtRun(status, tree);

        tree = std::make_unique<BT::Tree>(
                    factory->createTree(bt_id));
        status = BT::NodeStatus::IDLE;
  }
  std::cout<<"last stack executed" <<std::endl;



  return 0;
}

