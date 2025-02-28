#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include <geometry_msgs/PointStamped.h>
#include "bt_cpp/utils.h"

#include <condition_variable>
#include <mutex>

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "girona_utils/PursuitAction.h"

typedef actionlib::SimpleActionClient<girona_utils::PursuitAction> PursuitClient;

namespace chr = std::chrono;
namespace IauvGirona1000Survey {
// This is an asynchronous operation
class FollowPath : public BT::CoroActionNode {
  public:
   // Any TreeNode with ports must have a constructor with this signature
   FollowPath(ros::NodeHandle& nh, const std::string& name,
          const BT::NodeConfig& config)
       : CoroActionNode(name, config), 
       nh_(nh), 
       is_object_detected_(false), 
       radius_(0.3),
       prev_printed_msg_("empty"),
       timeout_server_msec_(500),
       is_node_started_{false} {
        ROS_INFO("constructor FollowPath");
       }

   // It is mandatory to define this static method.
   static BT::PortsList providedPorts() {
      return {BT::InputPort<std::string>("path_follow_is_completed"),
        BT::InputPort<std::string>("robot"),
        BT::InputPort<std::string>("survey_type") };
   }

   BT::NodeStatus onStart();
   BT::NodeStatus onRunning();
   
   void sendFollowPathGoal();
   void minimalDoneCallback(
       const actionlib::SimpleClientGoalState& state,
       const girona_utils::PursuitResultConstPtr& result);
   void feedbackCallback(const girona_utils::PursuitFeedbackConstPtr& feedback);
   BT::NodeStatus onResult(const girona_utils::PursuitResultConstPtr& res);

private:
  BT::NodeStatus tick() override final;
  unsigned timeout_server_msec_;
  SurveyType survey_type_;
  bool is_object_detected_;
  bool is_node_started_;
  ros::Subscriber object_pose_sub_;
  std::shared_ptr<PursuitClient> action_client_;
  ros::NodeHandle& nh_;
  std::string name_;
  std::string _robot_name;
  girona_utils::PursuitFeedback last_feedback_;
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
