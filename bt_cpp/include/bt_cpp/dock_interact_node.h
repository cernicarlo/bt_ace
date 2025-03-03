#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "bt_cpp/utils.h"

#include <condition_variable>
#include <mutex>

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "dual_gironas/DockingAction.h"

typedef actionlib::SimpleActionClient<dual_gironas::DockingAction> DockingClient;

namespace chr = std::chrono;
namespace IauvGirona1000Survey {
// This is an asynchronous operation
class DockInteract : public BT::CoroActionNode {
  public:
   // Any TreeNode with ports must have a constructor with this signature
   DockInteract(ros::NodeHandle& nh, const std::string& name,
          const BT::NodeConfig& config)
       : CoroActionNode(name, config), 
       nh_(nh),
       timeout_server_msec_(500),
       is_node_started_{false} {
        ROS_INFO("constructor DockInteract");
       }

   // It is mandatory to define this static method.
   static BT::PortsList providedPorts() {
      return {
        BT::InputPort<std::string>("robot"),
        BT::InputPort<std::string>("action") 
      };
   }

   BT::NodeStatus onStart();
   BT::NodeStatus onRunning();
   
   void sendDockInteractGoal();
   void minimalDoneCallback(
       const actionlib::SimpleClientGoalState& state,
       const dual_gironas::DockingResultConstPtr& result);
   void feedbackCallback(const dual_gironas::DockingFeedbackConstPtr& feedback);
   BT::NodeStatus onResult(const dual_gironas::DockingResultConstPtr& res);

private:
  BT::NodeStatus tick() override final;
  unsigned timeout_server_msec_;
  SurveyType survey_type_;
  bool is_object_detected_;
  bool is_node_started_;
  ros::Subscriber object_pose_sub_;
  std::shared_ptr<DockingClient> action_client_;
  ros::NodeHandle& nh_;
  std::string name_;
  std::string _robot_name;
  std::string _action;
  dual_gironas::DockingFeedback last_feedback_;
  int last_idx_waypoint_;
  std::chrono::steady_clock::time_point last_print_time_;
  float radius_;
  std::string _type;
  ros::ServiceClient _service_client;

  std::future<bool> _service_call_future;
  std::mutex object_pose_mutex_;
  std::condition_variable object_pose_cv_;
  bool is_object_pose_received_;
  bool is_request_initialized_;
  std::string prev_printed_msg_;
  std::string log_fp_;

};


BT::NodeStatus isPathClear();

}  // namespace IauvGirona1000Survey
