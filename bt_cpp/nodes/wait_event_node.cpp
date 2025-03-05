#include "bt_cpp/wait_event_node.h"
#include "behaviortree_cpp/bt_factory.h"

namespace IauvGirona1000Survey {

void WaitEvent::construction() {
  name_ = "WaitEvent";
  if (!getInput<std::string>("event_topic", event_topic_name_)) {
    throw BT::RuntimeError("missing required input [event_topic]");
  }
  event_sub_ = nh_.subscribe(event_topic_name_, 10, &WaitEvent::eventCb, this);
  ROS_INFO("waiting for %s event", event_topic_name_.c_str());


  std::cout << "Construction of " << name_ << std::endl;
}

void WaitEvent::eventCb(const std_msgs::Bool::ConstPtr& msg) {
  is_event_done_ = msg->data;
}

BT::NodeStatus WaitEvent::onStart() {
  // TODO: check if assigned by the user (check if you can assign from .xml)

  construction();
  ros::spinOnce();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitEvent::onRunning() {
  std::string log;
  double duration = 2.0;
  ros::spinOnce();

  // Check the service result periodically
  std::this_thread::sleep_for(chr::milliseconds(10));

  if (is_event_done_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitEvent::tick() {
  const BT::NodeStatus prev_status = status();

  if (prev_status == BT::NodeStatus::IDLE) {
    ROS_INFO("onStart..");
    BT::NodeStatus new_status = onStart();


    if (new_status == BT::NodeStatus::IDLE) {
      throw BT::LogicError("WaitEvent::onStart() must not return IDLE");
    }
    return new_status;
  }
  if (prev_status == BT::NodeStatus::RUNNING) {
    BT::NodeStatus new_status = onRunning();
    if (new_status == BT::NodeStatus::IDLE) {
      throw BT::LogicError("WaitEvent::onRunning() must not return IDLE");
    }
    return new_status;
  }
  return prev_status;
}

} // namespace IauvGirona1000Survey