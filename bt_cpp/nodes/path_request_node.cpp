#include "bt_cpp/path_request_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "iauv_motion_planner/GetPath.h"
#include "iauv_motion_planner/GetPathRequest.h"
#include "iauv_motion_planner/PlannerParam.h"

namespace IauvGirona1000Survey {
void PathRequest::construction() {
   name_ = "PathRequest";
   if (!getInput<std::string>("type", _type)) {
      throw BT::RuntimeError("missing required input [what]");
   }
   object_pose_sub_ = nh_.subscribe("/object_pose", 10, &PathRequest::objectPoseCallback, this);
   std::cout << "Construction of " << name_ << std::endl;
   last_print_time_ = std::chrono::steady_clock::now();

}


void PathRequest::objectPoseCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
   // useful only for circle to detect where to plan the object
   if (_type == "circular"){
      double duration = 2.0;
      std::stringstream log;
      log << "PatRequest - " <<_type << ": ";
      // object_pose_cv_.notify_one();
      _request.start.position.x = 3.0;
      _request.start.position.y = 3.0;
      _request.start.position.z = 5;
      _request.start.orientation.w = 1;
      log << " start position - set; ";
      if (!std::isnan(msg->point.x)){
         // Extract position data from the message
        
         _request.goal.position.x = msg->point.x;
         _request.goal.position.y = msg->point.y;
         _request.goal.position.z = msg->point.z;
         log << " goal position - set";
         is_object_pose_received_ = true;
      } else {
         log << " [WARNING] object not detected";
         _request.goal.position.x = 3;
         _request.goal.position.y = 4;
         _request.goal.position.z = 3;
         
      }
      _request.goal.orientation.w = 1;
      _request.goal.orientation.z = 0;
   
      printIfFromLastPrintHavePassedSomeSeconds(log.str(), duration);
      if (is_object_pose_received_){
         object_pose_sub_.shutdown();
      }
   }
   
   
    
}

bool PathRequest::hasEnoughTimePassed(double seconds) {
   auto now = std::chrono::steady_clock::now();
   auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
       now - last_print_time_);
   return elapsed.count() >=
          seconds * 1000;  // Convert seconds to milliseconds and compare
}

void PathRequest::printIfFromLastPrintHavePassedSomeSeconds(
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

BT::NodeStatus PathRequest::onStart() {
   // TODO: check if assigned by the user (check if you can assign from .xml)
   
   ROS_INFO("Path Request asking for construction");
   construction();

   ROS_INFO("[ PathRequest: SEND REQUEST ]. type of path=%s\n",
          _type.c_str());
   

   _service_client = nh_.serviceClient<iauv_motion_planner::GetPath>("/iauv_motion_planner/getPath");

   // Check if the service is available
   if (!_service_client.waitForExistence(ros::Duration(2.0))) {
      ROS_ERROR("[ PathRequest ] /iauv_motion_planner/getPath is not available");
      return BT::NodeStatus::FAILURE;
   }

   _request.header.frame_id = "world_ned";



   iauv_motion_planner::PlannerParam param;

   if (_type == "scan"){
       
      // TODO: pass start and goal request from outside
      _request.start.position.x = -4;
      _request.start.position.y = -4;
      _request.start.position.z = 5;
      _request.start.orientation.w = 1;

      _request.goal.position.x = 0;
      _request.goal.position.y = 0;
      _request.goal.position.z = 3;
      _request.goal.orientation.w = 1;
      _request.goal.orientation.z = 0;

      _request.planner = iauv_motion_planner::GetPathRequest::SCANNER;
      param.key = "width";
      param.value = "7";
      _request.params.push_back(param);
      param.key = "length";
      param.value = "5";
      _request.params.push_back(param);
      ROS_INFO("setting path scan");
      setOutput("survey_type", "scan");
   } else if (_type == "circular") {
       _request.planner = iauv_motion_planner::GetPathRequest::CIRCULAR;
       param.key = "radius";
       param.value = "4";
       _request.params.push_back(param);
       ROS_INFO("setting path circular");
       setOutput("survey_type", "circular");
   } else {
      ROS_ERROR("the type %s is not supported", _type.c_str());
      return BT::NodeStatus::FAILURE;
   }

   if (_type == "scan"){
      _service_call_future = std::async(std::launch::async, [&]() {
            return _service_client.call(_request, _response);
        });
   }
   ros::spinOnce();

   return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PathRequest::onRunning() {
   std::string log;
   double duration = 2.0;
   ros::spinOnce();

   // if circular, subscibe to object pose, set and send the request
   if (_type == "circular"){
      ros::spinOnce();
      if (!is_object_pose_received_ && !is_request_initialized_){
         log = "I didn't get yet object coords";
         printIfFromLastPrintHavePassedSomeSeconds(log, duration);
         
         return BT::NodeStatus::RUNNING;

      } else if (is_object_pose_received_ && !is_request_initialized_) {
         _service_call_future = std::async(std::launch::async, [&]() {
            return _service_client.call(_request, _response);
        });
        is_request_initialized_ = true;
        log = "object detected";
         printIfFromLastPrintHavePassedSomeSeconds(log, duration);
      }

   } 
        // Check the service result periodically
        std::this_thread::sleep_for(chr::milliseconds(10));

        if (_service_call_future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
            
            if (!_service_client.isValid()) {
               ROS_ERROR("Service client is not valid.");
            }

            ROS_INFO_STREAM("Start position: " << _request.start.position.x << ", " << _request.start.position.y << ", " << _request.start.position.z);
            ROS_INFO_STREAM("Goal position: " << _request.goal.position.x << ", " << _request.goal.position.y << ", " << _request.goal.position.z);
            if (!_service_call_future.valid()) {
               ROS_ERROR("Service call future is invalid.");
            }

            // Service call completed
            bool call_success = _service_call_future.get();
            if (call_success ) {
                ROS_INFO_STREAM("[ PathRequest: FINISHED ] - call_success:" << call_success);
                
                return BT::NodeStatus::SUCCESS;
            } else {
                ROS_INFO("[ PathRequest: FAILED ]");
                return BT::NodeStatus::FAILURE;
            }
        }

        return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PathRequest::tick() {
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
         throw BT::LogicError("PathRequest::onStart() must not return IDLE");
      }
      return new_status;
   }
   if (prev_status == BT::NodeStatus::RUNNING) {
      BT::NodeStatus new_status = onRunning();
      if (new_status == BT::NodeStatus::IDLE) {
         throw BT::LogicError(
             "PathRequest::onRunning() must not return IDLE");
      }
      return new_status;
   }
   return prev_status;
}

} // namespace