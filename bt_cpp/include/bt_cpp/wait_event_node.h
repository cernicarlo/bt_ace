#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "bt_cpp/utils.h"
#include <std_msgs/Bool.h>


#include <ros/ros.h>


namespace chr = std::chrono;

namespace IauvGirona1000Survey {

// This is an asynchronous operation
class WaitEvent : public BT::CoroActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    WaitEvent(ros::NodeHandle& nh, const std::string& name,
          const BT::NodeConfig& config)
       : CoroActionNode(name, config), 
       nh_(nh), 
       is_event_done_{false} {
        ROS_INFO("constructor");
       }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ 
         BT::InputPort<std::string>("event_topic")
         };
    }

   BT::NodeStatus onStart();
   BT::NodeStatus onRunning();
   void construction();
   void eventCb(const std_msgs::Bool::ConstPtr& msg);

  private:
    BT::NodeStatus tick() override final;
    unsigned timeout_server_msec_;
    std::string event_topic_name_;
    ros::Subscriber event_sub_;
    std::string name_;
    ros::NodeHandle& nh_;

    bool is_event_done_;
};

}  // namespace IauvGirona1000Survey
