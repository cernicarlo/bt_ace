#include "bt_cpp/follow_path_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "iauv_motion_planner/GetPath.h"
#include "iauv_motion_planner/GetPathRequest.h"
#include "iauv_motion_planner/PlannerParam.h"

namespace IauvGirona1000Survey {

BT::NodeStatus FollowPath::onStart() {
   name_ = "FollowPath";
   log_fp_ = "[FollowPath] : ";

   std::string survey_type_str;
   if (getInput<std::string>("survey_type", survey_type_str)) {
      std::string log = log_fp_ + "Survey type set to ";
      if (survey_type_str == "scan") {
         survey_type_ = SCAN;
         ROS_INFO("%s", (log + "SCAN").c_str());
      } else if (survey_type_str == "circular") {
         survey_type_ = CIRCULAR;
         ROS_INFO("%s", (log + "CIRCULAR").c_str());
      } else if (survey_type_str == "simple") {
         survey_type_ = SIMPLE;
         ROS_INFO("%s", (log + "SIMPLE").c_str());
      }  else {
         throw BT::RuntimeError("Invalid survey type");
      }
   } else {
      throw BT::RuntimeError("missing required input [survey_type]");
   }

   std::cout << "Construction of " << name_ << std::endl;
   
   last_idx_waypoint_ = -1;

   sendFollowPathGoal();
   // sendBtGoal();
   last_print_time_ = std::chrono::steady_clock::now();
   return BT::NodeStatus::RUNNING;
}

void FollowPath::sendFollowPathGoal() {
   girona_utils::PursuitGoal goal;
    // Wait for the message with a timeout of 1 second
    boost::shared_ptr<nav_msgs::Path const> path_msg = ros::topic::waitForMessage<nav_msgs::Path>("/iauv_motion_planner/path", nh_, ros::Duration(1.0));

    // Check if the message was received (not null)
    if (path_msg) {
        // Dereference the shared pointer and assign the message
        goal.path = *path_msg;
        goal.radius = radius_;

        // Send the goal to the action client
        action_client_->sendGoal(
            goal, boost::bind(&FollowPath::minimalDoneCallback, this, _1, _2),
            PursuitClient::SimpleActiveCallback(),
            boost::bind(&FollowPath::feedbackCallback, this, _1)
        );
    } else {
        ROS_WARN("No path message received within the timeout.");
    }
}


void FollowPath::minimalDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const girona_utils::PursuitResultConstPtr& result) {
   //  const bt_policy::BtResultConstPtr& result) {
   
   std::cout << log_fp_ << name_ << ": Result from server: "
             << (result->success ? "Success" : "Failure")
             << " with state: " << state.toString() << std::endl;
}

void FollowPath::feedbackCallback(
   const girona_utils::PursuitFeedbackConstPtr& feedback) {
   //  const bt_policy::BtFeedbackConstPtr& feedback) {
   last_feedback_ = *feedback;
}


BT::NodeStatus FollowPath::onRunning() {
   // if (isTimeOutReached()) {
   //    return BT::NodeStatus::FAILURE;
   // }
   auto action_state = action_client_->getState();

   if (action_state == actionlib::SimpleClientGoalState::PENDING ||
       action_state == actionlib::SimpleClientGoalState::ACTIVE) {
      
      // print only if it reached a new waypoint
      if (last_feedback_.waypoint > last_idx_waypoint_){
         std::string state_server =
          action_state == actionlib::SimpleClientGoalState::PENDING ? "PENDING"
                                                                    : "ACTIVE";
         std::string position_str = std::to_string(last_feedback_.waypoint);

         std::string msg = "[" + name_ + "] : " + state_server +
                           ", waypoint: " + position_str;
         printIfFromLastPrintHavePassedSomeSeconds(msg, 1.0,prev_printed_msg_,last_print_time_);
         last_idx_waypoint_ = last_feedback_.waypoint;
      };
      
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

BT::NodeStatus FollowPath::onResult(
    const girona_utils::PursuitResultConstPtr& res) {
   //  const bt_policy::BtResultConstPtr& res) {

   if (!res->success) {
      ros::spinOnce();

      return BT::NodeStatus::FAILURE;
   };
   return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus FollowPath::tick() {
   const BT::NodeStatus prev_status = status();
   ros::Duration server_timeout(static_cast<double>(timeout_server_msec_) *
                                1e-3);
   bool connected = action_client_->waitForServer(server_timeout);
   if (!connected) {
      ROS_WARN("timeout to connect to server (missing server)");
      return BT::NodeStatus::FAILURE;
      // return onFailedRequest(MISSING_SERVER);
   }

   if (prev_status == BT::NodeStatus::IDLE) {
      BT::NodeStatus new_status = onStart();
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