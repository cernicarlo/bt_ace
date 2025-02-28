#include "bt_cpp/dock_interact_node.h"
#include "behaviortree_cpp/bt_factory.h"

namespace IauvGirona1000Survey {

BT::NodeStatus DockInteract::onStart() {
   name_ = "DockInteract";
   log_fp_ = "[DockInteract] : ";


   if (!getInput<std::string>("action", _action)) {
      throw BT::RuntimeError("missing required input [action]");
   }

   if (!getInput<std::string>("robot", _robot_name)) {
      throw BT::RuntimeError("missing required input [robot]");
   }

   std::string action_client_name = "/docking_action_" + _robot_name;
   ROS_INFO("initializing action_client_");
   action_client_ =
      std::make_shared<DockingClient>(
         nh_, action_client_name, true);
   
   
   ROS_INFO("initialized action_client_ (%s)", action_client_name.c_str());
  ros::Duration server_timeout(static_cast<double>(timeout_server_msec_) *
                                1e-3);
   bool connected = action_client_->waitForServer(server_timeout);
   
   if (!connected) {
      ROS_WARN("dock: timeout to connect to server (missing server)");
      return BT::NodeStatus::FAILURE;
      // return onFailedRequest(MISSING_SERVER);
   }

   std::cout << "Construction of " << name_ << std::endl;

   sendDockInteractGoal();
   // sendBtGoal();
   last_print_time_ = std::chrono::steady_clock::now();
   return BT::NodeStatus::RUNNING;
}

void DockInteract::sendDockInteractGoal() {
   dual_gironas::DockingGoal goal;
   goal.action = _action;

    // Check if the message was received (not null)
   //  if (path_msg) {
        // Dereference the shared pointer and assign the message

   // Send the goal to the action client
   action_client_->sendGoal(
      goal, boost::bind(&DockInteract::minimalDoneCallback, this, _1, _2),
      DockingClient::SimpleActiveCallback(),
      boost::bind(&DockInteract::feedbackCallback, this, _1)
   );

   //  } else {
   //      ROS_WARN("No action message received within the timeout.");
   //  }
}


void DockInteract::minimalDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const dual_gironas::DockingResultConstPtr& result) {
   //  const bt_policy::BtResultConstPtr& result) {
   
   std::cout << log_fp_ << name_ << ": Result from server: "
             << (result->success ? "Success" : "Failure")
             << " with state: " << state.toString() << std::endl;
}

void DockInteract::feedbackCallback(
   const dual_gironas::DockingFeedbackConstPtr& feedback) {
   last_feedback_ = *feedback;
   ROS_INFO("docking status: (%s)", last_feedback_.status.c_str());
}


BT::NodeStatus DockInteract::onRunning() {
   // if (isTimeOutReached()) {
   //    return BT::NodeStatus::FAILURE;
   // }
   auto action_state = action_client_->getState();

   if (action_state == actionlib::SimpleClientGoalState::PENDING ||
       action_state == actionlib::SimpleClientGoalState::ACTIVE) {
      
      // print only if it reached a new waypoint
      // if (last_feedback_.waypoint > last_idx_waypoint_){
      std::string state_server =
         action_state == actionlib::SimpleClientGoalState::PENDING ? "PENDING"
                                                                  : "ACTIVE";

      std::string msg = "[" + name_ + "] : " + state_server +
                        ", robot: " + _robot_name + " executing " + _action;
      printIfFromLastPrintHavePassedSomeSeconds(msg, 1.0, prev_printed_msg_, last_print_time_);
      // };
      
      sleep(0.5);
      return BT::NodeStatus::RUNNING;
   } else if (action_state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return onResult(action_client_->getResult());
   } else if (action_state == actionlib::SimpleClientGoalState::ABORTED) {
      std::cout << "[" << name_ << ": ABORTED_BY_SERVER"
                << std::endl;
      return BT::NodeStatus::FAILURE;
   } else if (action_state == actionlib::SimpleClientGoalState::REJECTED) {
      std::cout << "[" << name_ << ": REJECTED_BY_SERVER"
                << std::endl;
      return BT::NodeStatus::FAILURE;
   } else {
      // FIXME: is there any other valid state we should consider?
      throw std::logic_error("Unexpected state in RosActionNode::tick()");
   }
}

BT::NodeStatus DockInteract::onResult(
    const dual_gironas::DockingResultConstPtr& res) {
   //  const bt_policy::BtResultConstPtr& res) {

   if (!res->success) {
      ros::spinOnce();

      return BT::NodeStatus::FAILURE;
   };
   return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus DockInteract::tick() {
   const BT::NodeStatus prev_status = status();

   if (prev_status == BT::NodeStatus::IDLE && !is_node_started_) {
      ROS_INFO("onStart()");
      BT::NodeStatus new_status = onStart();
      ROS_INFO("done initialized action_client_");

      is_node_started_ = true;
      

      if (new_status == BT::NodeStatus::IDLE) {
         throw BT::LogicError("HelpSeekerNode::onStart() must not return IDLE");
      }
      return new_status;
   }

   if (prev_status == BT::NodeStatus::RUNNING) {
      BT::NodeStatus new_status = onRunning();
      if (new_status == BT::NodeStatus::IDLE) {
         throw BT::LogicError(
             "HelpSeekerNode::onRunning() must not return IDLE");
      }
      return new_status;
   }
   return prev_status;
}


}