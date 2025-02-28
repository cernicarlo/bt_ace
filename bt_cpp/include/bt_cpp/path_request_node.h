#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "iauv_motion_planner/GetPath.h"
#include "iauv_motion_planner/GetPathRequest.h"
#include "iauv_motion_planner/PlannerParam.h"
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
namespace BT{
template <>
inline IauvGirona1000Survey::Pose3D BT::convertFromString(BT::StringView key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if(parts.size() != 3)
  {
    throw BT::RuntimeError("invalid input)");
  }
  else
  {
    IauvGirona1000Survey::Pose3D output;
    output.x = BT::convertFromString<float>(parts[0]);
    output.y = BT::convertFromString<float>(parts[1]);
    output.z = BT::convertFromString<float>(parts[2]);
    return output;
  }
}
}

namespace IauvGirona1000Survey {

// This is an asynchronous operation
class PathRequest : public BT::CoroActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    PathRequest(ros::NodeHandle& nh, const std::string& name,
          const BT::NodeConfig& config)
       : CoroActionNode(name, config), 
       nh_(nh), 
       is_object_pose_received_(false), 
       is_request_initialized_(false),
       prev_printed_msg_("empty"),
       timeout_server_msec_(500),
       is_node_started_{false} {
        ROS_INFO("constructor");
       }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ 
         BT::InputPort<std::string>("type"),
         BT::InputPort<std::string>("robot"),
         BT::InputPort<Pose3D>("start"),
         BT::InputPort<Pose3D>("goal"),
         BT::InputPort<bool>("is_to_object"),
         BT::InputPort<std::string>("width"),
         BT::InputPort<std::string>("length"),
         BT::InputPort<std::string>("radius"),
         BT::InputPort<float>("w_start"),
         BT::InputPort<float>("z_start"),
         BT::InputPort<float>("w_end"),
         BT::InputPort<float>("z_end"),
         BT::OutputPort<std::string>("survey_type")
         };
    }

   BT::NodeStatus onStart();
   BT::NodeStatus onRunning();
   void construction();
   void objectPoseCallback(const geometry_msgs::PointStamped::ConstPtr& msg);

  private:
    BT::NodeStatus tick() override final;
    unsigned timeout_server_msec_;
    std::string _type;
    std::string _robot_name;
    std::string name_;
    ros::ServiceClient _service_client;
    iauv_motion_planner::GetPathRequest _request;
    iauv_motion_planner::GetPathResponse _response;
    std::future<bool> _service_call_future;
    SurveyType survey_type_;
    std::mutex object_pose_mutex_;
    std::condition_variable object_pose_cv_;
    bool is_object_pose_received_;
    bool is_request_initialized_;
    bool is_node_started_;
    ros::Subscriber object_pose_sub_;
    std::shared_ptr<PursuitClient> action_client_;
    ros::NodeHandle& nh_;
    girona_utils::PursuitFeedback last_feedback_;
    int last_idx_waypoint_;
    std::string prev_printed_msg_;
    std::chrono::steady_clock::time_point last_print_time_;
    Pose3D _goal;
    Pose3D _start;
    std::string _width;
    std::string _length;
    std::string _radius;
    std::string _param;
    float _w_start;
    float _z_start;
    float _w_end;
    float _z_end;

    bool _is_to_object;
};

}  // namespace IauvGirona1000Survey
