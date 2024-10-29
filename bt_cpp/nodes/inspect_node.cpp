#include "bt_cpp/inspect_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "iauv_motion_planner/GetPath.h"
#include "iauv_motion_planner/GetPathRequest.h"
#include "iauv_motion_planner/PlannerParam.h"

namespace IauvGirona1000Survey {

BT::NodeStatus Inspect::onStart() {
   name_ = "Inspect";

   std::string survey_type_str;
   if (getInput<std::string>("survey_type", survey_type_str)) {
      if (survey_type_str == "scan") {
         survey_type_ = SCAN;
         ROS_INFO("Survey type set to SCAN");
      } else if (survey_type_str == "circular") {
         survey_type_ = CIRCULAR;
         ROS_INFO("Survey type set to CIRCULAR");
      } else {
         throw BT::RuntimeError("Invalid survey type");
      }
   } else {
      throw BT::RuntimeError("missing required input [survey_type]");
   }

   object_pose_sub_ = nh_.subscribe("/object_pose", 10, &Inspect::objectPoseCallback, this);

   std::cout << "Construction of " << name_ << std::endl;
   
   last_idx_waypoint_ = -1;

   sendFollowPathGoal();
   // sendBtGoal();
   last_print_time_ = std::chrono::steady_clock::now();
   return BT::NodeStatus::RUNNING;
}

void Inspect::sendFollowPathGoal() {
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
            goal, boost::bind(&Inspect::minimalDoneCallback, this, _1, _2),
            PursuitClient::SimpleActiveCallback(),
            boost::bind(&Inspect::feedbackCallback, this, _1)
        );
    } else {
        ROS_WARN("No path message received within the timeout.");
    }
}


void Inspect::minimalDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const girona_utils::PursuitResultConstPtr& result) {
   //  const bt_policy::BtResultConstPtr& result) {
   
   std::cout << "[PathRequest]:" << name_ << ": Result from server: "
             << (result->success ? "Success" : "Failure")
             << " with state: " << state.toString() << std::endl;
}

void Inspect::feedbackCallback(
   const girona_utils::PursuitFeedbackConstPtr& feedback) {
   //  const bt_policy::BtFeedbackConstPtr& feedback) {
   last_feedback_ = *feedback;
}



void Inspect::objectPoseCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
   // this is used only for scan to check when you detect something
   if (survey_type_ == SCAN){
      std::stringstream log;
      log << "SCAN: ";

      if (!std::isnan(msg->point.x)){
         is_object_detected_ = true;
         log << "Received object position - x: " << msg->point.x << ", y: " << msg->point.y << ", z: " << msg->point.z;
         printIfFromLastPrintHavePassedSomeSeconds(log.str(), 1.0);
      } else {
         is_object_detected_ = false;
      }
   }   
    
}



bool Inspect::hasEnoughTimePassed(double seconds) {
   auto now = std::chrono::steady_clock::now();
   auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
       now - last_print_time_);
   return elapsed.count() >=
          seconds * 1000;  // Convert seconds to milliseconds and compare
}

void Inspect::printIfFromLastPrintHavePassedSomeSeconds(
    const std::string& msg, double seconds) {
   if (msg != prev_printed_msg_) {
      std::cout << msg << std::endl;
      prev_printed_msg_ = msg;
      return;
   }

   if (hasEnoughTimePassed(seconds)) {
      std::cout << msg << std::endl;
      last_print_time_ =
          std::chrono::steady_clock::now();  // Update the last print time
   }
}

BT::NodeStatus Inspect::onRunning() {
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
         msg +=  " - simulating blindness: path clear";
         printIfFromLastPrintHavePassedSomeSeconds(msg);
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

BT::NodeStatus Inspect::onResult(
    const girona_utils::PursuitResultConstPtr& res) {
   //  const bt_policy::BtResultConstPtr& res) {

   if (!res->success) {
      ros::spinOnce();

      return BT::NodeStatus::FAILURE;
   };
   return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus Inspect::tick() {
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