/**
 * Check for BT provided by the LLM. The Possible Nodes running need to be registered in the main (like it's done for LogSequence).
 * The LLm has to write its new BT in the folder `llm_bt_xml` in the appropriate format (for valid examples, please take a look at the BTs in `bt_xml` folder)
 * To execute one last BT before concluding the mission, the bt file name has to be `last_bt.xml`
 */


#include "behaviortree_cpp/bt_factory.h"
#include "bt_cpp/bt_manager.h"
#include "bt_cpp/point_cloud_processor.h"
#include "bt_cpp/follow_path_node.h"
#include "bt_cpp/path_request_node.h"
#include "bt_cpp/dock_interact_node.h"
#include "bt_cpp/is_path_clear_node.h"
#include "bt_cpp/log_nodes.h"
#include "bt_cpp/primitive_detection_client.h"
#include "bt_cpp/object_sorter.h"
#include "bt_cpp/utils.h"
#include "bt_cpp/mission_utils.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <std_srvs/Trigger.h>

#include <ros/package.h>

using namespace BT;

int main(int argc, char** argv)
{
/*
  variables initialization
*/ 
  // ROS init
  ros::init(argc, argv, "llm_bt_cpp_b_node");
  ros::NodeHandle nh;

  // path init
  std::string xml_path = ros::package::getPath("bt_cpp") + "/llm_bt_xml_b/";

  // Factory init
  std::unique_ptr<BT::BehaviorTreeFactory> factory;
  factory = std::make_unique<BT::BehaviorTreeFactory>();
  // initialize possible BT nodes
  registerCustomNode<IauvGirona1000Survey::PathRequest>(
      *factory, "PathRequest", nh);
  registerCustomNode<IauvGirona1000Survey::FollowPath>(
      *factory, "FollowPath", nh);
  registerCustomNode<IauvGirona1000Survey::DockInteract>(
    *factory, "DockInteract", nh);
  factory->registerNodeType<isPathClear>("isPathClear", "is_path_clear", nh);
  PortsList say_something_ports = { InputPort<std::string>("message") };
  factory->registerSimpleAction("LogSequence", LogSequence, say_something_ports);
  factory->registerSimpleAction("LogFallback", LogFallback, say_something_ports);

  // BTManager
  BTManager bt_manager(xml_path, nh, *factory);
  std::vector<std::string> action_targets;


  /*
    Mission:
      Execute whatever there is in the llm_bt_xml
      To conclude the mission, insert a bt named like the variable last_bt
      which will execute it and finish the mission
  */
  bool is_mission_on = true;
  std::string last_bt = "last_bt.xml";
  while (is_mission_on && ros::ok()) {
    bool is_bt_found = false;
    std::filesystem::path bt_file_path;
    
    // Iterate through the files in the directory
    for (const auto& entry : std::filesystem::directory_iterator(xml_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".xml") {
            bt_file_path = entry.path();
            std::string bt_filename = entry.path().filename();
            std::cout << "[robotB] BT file found: " << bt_filename << std::endl;
            is_bt_found = true;

            if (bt_filename == last_bt) {
              is_mission_on = false;
            }

            // Get the filename without the extension
            std::string bt_id = bt_file_path.stem().string();
            action_targets.push_back(bt_id);
            break;
        }
    }

    if (is_bt_found) {
      // Start mission taking BT from mission file
      bt_manager.populateStack(action_targets);


      /// Execute stack mission
      bt_manager.executeStack();

      action_targets.clear();

      // Construct new filename with .executed extension
      std::filesystem::path executed_bt_path = bt_file_path;
      executed_bt_path += ".executed";

      // Rename the file
      std::filesystem::rename(bt_file_path, executed_bt_path);
      std::cout << "[robotB] Renamed to: " << executed_bt_path.filename() << std::endl;
        
    } else {
      std::cout << "[robotB] No BT file found. Checking again in 1 second..." << std::endl;
    }

    // Wait for 1 second before checking again
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

std::cout << "[robotB] Mission completed" << std::endl;

  return 0;
}

