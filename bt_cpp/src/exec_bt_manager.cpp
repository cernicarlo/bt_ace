#include "behaviortree_cpp/bt_factory.h"
#include "bt_cpp/bt_manager.h"
#include "bt_cpp/point_cloud_processor.h"
#include "bt_cpp/follow_path_node.h"
#include "bt_cpp/path_request_node.h"
#include "bt_cpp/is_path_clear_node.h"
#include "bt_cpp/log_nodes.h"
#include "bt_cpp/primitive_detection_client.h"
#include "bt_cpp/object_sorter.h"
#include "bt_cpp/utils.h"
#include "bt_cpp/mission_utils.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <std_srvs/Trigger.h>

#include <ros/package.h>

using namespace BT;

void log_sorted_obj(const std::vector<LabeledObject> sorted_objects){
  ROS_INFO("Objects sorted by distance from robot (luma_station):");
  for (const auto& obj : sorted_objects) {
    // Convert actions vector to a single string
    std::ostringstream actions_stream;
    for (const auto& action : obj.info.actions) {
        actions_stream << action;
        if (&action != &obj.info.actions.back()) {  // Add a separator between actions
            actions_stream << ", ";
        }
    }
    std::string actions_str = actions_stream.str();

    ROS_INFO("Label: %s, Actions: %s, Distance: %.2f", obj.label.c_str(), actions_str.c_str(), obj.distance);
  }};


void log_all_detected_object(std::unordered_map<std::string, ObjectInfo> detected_objects){

  ROS_INFO("All detected objects with actions:");
  for (const auto& item : detected_objects) {
      const std::string& label = item.first;
      const ObjectInfo& obj_info = item.second;

      ROS_INFO("Label: %s", label.c_str());
      ROS_INFO("Surface Point: x=%.2f, y=%.2f, z=%.2f", obj_info.surface_point.x, obj_info.surface_point.y, obj_info.surface_point.z);
      ROS_INFO("Actions:");
      for (const auto& action : obj_info.actions) {
          ROS_INFO(" - %s", action.c_str());
      }
  }

}

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
  std::vector<std::string> action_targets;

  // Script variable
  int num_clouds;
  std::unordered_map<std::string, ObjectInfo>  detected_objects;
  geometry_msgs::Point luma_station;
  luma_station.x = -10.0;
  luma_station.y = 3.0;
  luma_station.z = 6.25;


/*
  Mission
*/ 

  /* 
  1st - execute main mission (survey) 
  */

  // Validate mission
  if (!validateMission(nh)) {
      return 1;
  }

  // Create mission stack
  if (!createMissionStack(nh, action_targets)) {
      return 1;
  }

  // Start mission taking BT from mission file
  bt_manager.populateStack(action_targets);


/// Execute stack mission
  bt_manager.executeStack();
  std::cout << "First stack executed" << std::endl;


  /* 
  2nd - communicate to user through luma what seen 
  */

// pd_client lifetime begin
{  
  // activate pcl_processor (it will trigger the clustering service as soon as luma pubblication happens)
  // after that, the labeled obje info will be published
  // pd_client will subscribe, get_actions and return the detected objects
  // PointCloudProcessor pc_processor(nh);
  PrimitiveDetectionClient pd_client(nh);
  // one sec to initialize pd_client and subscribe
  ros::Time start_time = ros::Time::now();
  ros::Duration duration(1.0); // 1 second duration

  while (ros::Time::now() - start_time < duration) {
      ros::spinOnce();
      ros::Duration(0.01).sleep(); // sleep for 10 ms between each spinOnce
  }

/// go to Luma if not already there
  if(!pd_client.is_luma_detected_){

    bt_manager.populateStack({"goto_luma"});
    // TODO: introduce the condition in executeStack
    bt_manager.executeStack();
    std::cout << "Go to LUMA executed" << std::endl;
  } else {
    std::cout << "AUV already at LUMA" << std::endl;
  }
    
  
  
  std::cout<<"processing all the primitive detected"<<std::endl;
  spinUntilCondition([&]() { return pd_client.isProcessingComplete(); });


  // Retrieve and display all detected objects
  detected_objects = pd_client.getAllDetectedObjects();

} // end lifetime pd_client lifetime

/// sort the objects according to their distance wrt the robot
 // we assume the current pose of the robot is the same of the luma. (TODO: it must be girona)
 ObjectSorter sorter(luma_station);
 sorter.sortByDistance(detected_objects);

 const auto& sorted_objects = sorter.getSortedObjects();
 log_sorted_obj(sorted_objects);



  /*
  3rd part of mission, execute new stack
  */
  ROS_INFO("start mission from luma");
  // we will start the mission from the luma position, assuming the robot is there
  std::string start_pos_mission = IauvGirona1000Survey::pointToString(luma_station);
  // repopulate the stack with object info
  bt_manager.populateSortedObjectsStackFromSorter(sorted_objects, start_pos_mission);
  bt_manager.printSortedObjectsStack();

  bt_manager.executeDynamicStack();
  std::cout << "Last stack executed" << std::endl;

  return 0;
}

