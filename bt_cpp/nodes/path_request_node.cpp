#include "bt_cpp/path_request_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "iauv_motion_planner/GetPath.h"
#include "iauv_motion_planner/GetPathRequest.h"
#include "iauv_motion_planner/PlannerParam.h"

namespace IauvGirona1000Survey {
void PathRequest::construction() {
   name_ = "PathRequest";
   if (!getInput<std::string>("type", _type)) {
      throw BT::RuntimeError("missing required input [type]");
   }
   if (!getInput<Pose3D>("start", _start)) {
      throw BT::RuntimeError("missing required input [start]");
   }

   if (!getInput<std::string>("robot", _robot_name)) {
      throw BT::RuntimeError("missing required input [start]");
   }
   std::string action_client_name =  _robot_name + "/pursuit_controller";
   ROS_INFO("initializing action_client_. action_client_name: %s", action_client_name.c_str());
   action_client_ =
      std::make_shared<PursuitClient>(
         nh_, action_client_name, true);
   ROS_INFO("initialized action_client_");

   auto is_to_object = getInput<bool>("is_to_object");
   if(!is_to_object)
   {
   throw BT::RuntimeError("missing required input [is_to_object]");
   }
   _is_to_object = is_to_object.value();


   if (!getInput<Pose3D>("goal", _goal) && !getInput<bool>("is_to_object")) {
      throw BT::RuntimeError("missing required input [goal] or [is_to_object]");
   } else if (getInput<Pose3D>("goal", _goal) && !getInput<bool>("is_to_object")){
      std::cout<<"going to goal"<<_goal.x<<", "<<_goal.y<<", "<<_goal.z<<std::endl;
   } else {
      std::cout<<"going to the detected object"<<std::endl;
      auto is_to_object = getInput<bool>("is_to_object");
      _is_to_object = is_to_object.value();
      object_pose_sub_ = nh_.subscribe("/object_pose", 10, &PathRequest::objectPoseCallback, this);
   }


   if (_type == "scan"){
      if (!getInput<std::string>("length", _length)) {
         throw BT::RuntimeError("missing required input [length]");
      }
      if (!getInput<std::string>("width", _width)) {
         throw BT::RuntimeError("missing required input [width]");
      }
   } else if(_type == "circular"){
      if (!getInput<std::string>("radius", _radius)) {
         throw BT::RuntimeError("missing required input [radius]");
      }
   } 
   else if(_type == "simple"){
      auto w_start = getInput<float>("w_start").value();
      if(!w_start)
      {
      throw BT::RuntimeError("missing required input [w_start]");
      }
      _w_start = w_start;

      auto w_end = getInput<float>("w_end").value();
      if(!w_end)
      {
      throw BT::RuntimeError("missing required input [w_end]");
      }
      _w_end = w_end;

      auto z_start = getInput<float>("z_start").value();
      if(!z_start)
      {
      throw BT::RuntimeError("missing required input [z_start]");
      }
      _z_start = z_start;

      auto z_end = getInput<float>("z_end").value();
      if(!w_start)
      {
      throw BT::RuntimeError("missing required input [z_end]");
      }
      _z_end = z_end;
   }

   
   std::cout << "Construction of " << name_ << std::endl;
   last_print_time_ = std::chrono::steady_clock::now();

}


void PathRequest::objectPoseCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
   // useful only for circle to detect where to plan the object
   if (_is_to_object){
      double duration = 2.0;
      std::stringstream log;
      log << "PatRequest - " <<_type << " set wrt object: ";
      // object_pose_cv_.notify_one();
      _request.start.position.x = _start.x; //3.0
      _request.start.position.y = _start.y; //3.0
      _request.start.position.z = _start.z; //5.0
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
         _request.goal.position.x = _goal.x; //3.0
         _request.goal.position.y = _goal.y; //4.0
         _request.goal.position.z = _goal.z; //3.0
         
      }
      _request.goal.orientation.w = 1;
      _request.goal.orientation.z = 0;
   
      printIfFromLastPrintHavePassedSomeSeconds(log.str(), duration,prev_printed_msg_,last_print_time_);
      if (is_object_pose_received_){
         // position set. we can shut it down
         object_pose_sub_.shutdown();
      }
   }
   
   
    
}


BT::NodeStatus PathRequest::onStart() {
   // TODO: check if assigned by the user (check if you can assign from .xml)
   
   ROS_INFO("Path Request asking for construction");
   construction();

   ROS_INFO("[ PathRequest: SEND REQUEST ]. type of path=%s\n",
          _type.c_str());
   
   std::string service_name = "/" + _robot_name + "/iauv_motion_planner/getPath";
   _service_client = nh_.serviceClient<iauv_motion_planner::GetPath>(service_name);

   // Check if the service is available
   if (!_service_client.waitForExistence(ros::Duration(2.0))) {
      ROS_ERROR("[ PathRequest ] %s is not available", service_name.c_str());
      return BT::NodeStatus::FAILURE;
   }

   _request.header.frame_id = "world_ned";
   _request.start.position.x = _start.x; //-4.0
   _request.start.position.y = _start.y; //-4.0
   _request.start.position.z = _start.z; //5.0
   _request.start.orientation.w = 1;

   if (!_is_to_object){
      // set goal from blackboard only if it's not wrt object
      _request.goal.position.x = _goal.x; //0.0
      _request.goal.position.y = _goal.y; //0.0
      _request.goal.position.z = _goal.z; //3.0

   }
   
   _request.goal.orientation.w = 1;
   _request.goal.orientation.z = 0;

   iauv_motion_planner::PlannerParam param;
   if (_type == "scan"){
      _request.planner = iauv_motion_planner::GetPathRequest::SCANNER;
      param.key = "width";
      param.value = _width;//"7";
      _request.params.push_back(param);
      param.key = "length";
      param.value = _length;//"5";
      _request.params.push_back(param);
      ROS_INFO("setting path scan");
      setOutput("survey_type", "scan");
   } else if (_type == "circular") {
       _request.planner = iauv_motion_planner::GetPathRequest::CIRCULAR;
       param.key = "radius";
       param.value = _radius;//"4";
       _request.params.push_back(param);
       ROS_INFO("setting path circular");
       setOutput("survey_type", "circular");
   } else if (_type == "simple") {
      _request.planner = iauv_motion_planner::GetPathRequest::SIMPLE;
      _request.start.orientation.w = _w_start;
      _request.start.orientation.z = _z_start;
      _request.goal.orientation.w = _w_end;
      _request.goal.orientation.z = _z_end;
      ROS_INFO("setting path simple");
      setOutput("survey_type", "simple");
   } else {
      ROS_ERROR("the type %s is not supported", _type.c_str());
      return BT::NodeStatus::FAILURE;
   }

   
   _service_call_future = std::async(std::launch::async, [&]() {
            return _service_client.call(_request, _response);
        });
   ros::spinOnce();

   return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PathRequest::onRunning() {
   std::string log;
   double duration = 2.0;
   ros::spinOnce();

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

   if (prev_status == BT::NodeStatus::IDLE && !is_node_started_) {
      ROS_INFO("onStart..");
      BT::NodeStatus new_status = onStart();

      ros::Duration server_timeout(static_cast<double>(timeout_server_msec_) *
                                1e-3);
      bool connected = action_client_->waitForServer(server_timeout);
      is_node_started_ = true;

      if (!connected) {
         ROS_WARN("path request: timeout to connect to server (missing server)");
         return BT::NodeStatus::FAILURE;
         // return onFailedRequest(MISSING_SERVER);
      }

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