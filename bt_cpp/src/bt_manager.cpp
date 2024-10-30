#include "behaviortree_cpp/bt_factory.h"
#include "bt_cpp/inspect_node.h"
#include "bt_cpp/path_request_node.h"
#include "bt_cpp/is_path_clear_node.h"
#include "bt_cpp/log_nodes.h"
#include <fstream>
#include <iostream>
#include <cmath>

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
  ros::init(argc, argv, "bt_cpp_node");

  BT::NodeStatus status = BT::NodeStatus::IDLE;

  std::string abs_repo_path = "/home/ros/catkin_ws/src/";
  std::string relative_xml_fold_path = "bt_ace/bt_cpp/bt_xml/";
  
  // BehaviorTreeFactory factory;

  std::unique_ptr<BT::BehaviorTreeFactory> factory;
  std::string path;
  ros::NodeHandle nh;
  // path = "/home/ros/catkin_ws/src/bt_ace/bt_cpp/bt_xml/iauv_girona1000_survey_circular_true.xml";

  std::deque<std::string> stack;

  // TODO: function to populate stack
  std::string bt_exec = "BT_1.xml";
  stack.push_back(bt_exec);
  bt_exec = "BT_2.xml";
  stack.push_back(bt_exec);
  bt_exec = "BT_3.xml";
  stack.push_back(bt_exec);
  
  
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
  //         std::bind(IauvGirona1000Survey::Inspect::isPathClear));

  PortsList say_something_ports = { InputPort<std::string>("message") };
  factory->registerSimpleAction("LogSequence", LogSequence, say_something_ports);
  factory->registerSimpleAction("LogFallback", LogFallback, say_something_ports);

  // factory->registerNodeType<DummyNodes::SaySomething>("SaySomething");
  registerCustomNode<IauvGirona1000Survey::Inspect>(
      *factory, "Inspect", nh);
   
  
  // factory.registerNodeType<SaySomething>("SaySomething");
  // factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");

  // Parse the XML file and create a tree_ from it
  std::unique_ptr<BT::Tree> tree;
  std::string bt_id = "main_bt";



    while(!stack.empty()) {

        std::string xml_tree = stack.front();
        stack.pop_front();
        path = abs_repo_path + relative_xml_fold_path + xml_tree;
        factory->registerBehaviorTreeFromFile(path);
        tree = std::make_unique<BT::Tree>(
            factory->createTree(bt_id));
        BT::NodeStatus status = BT::NodeStatus::IDLE;

        BtRun(status, tree);

        tree = std::make_unique<BT::Tree>(
                    factory->createTree(bt_id));
    }


  return 0;
}

