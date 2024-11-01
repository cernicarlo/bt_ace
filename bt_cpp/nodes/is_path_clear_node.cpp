
#include "bt_cpp/is_path_clear_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "iauv_motion_planner/GetPath.h"
#include "iauv_motion_planner/GetPathRequest.h"
#include "iauv_motion_planner/PlannerParam.h"
#include "bt_cpp/utils.h"

void isPathClear::objectPoseCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        if (survey_type_ == IauvGirona1000Survey::SCAN)
        {
            std::stringstream log;
            log << "SCAN: ";
            if (!std::isnan(msg->point.x))
            {
              log << "Received object position - x: " << msg->point.x
                    << ", y: " << msg->point.y
                    << ", z: " << msg->point.z;
                if((msg->point.x != prev_x_) && (msg->point.x != prev_y_) && (msg->point.x != prev_x_))
                {
                  is_object_detected_ = true;
                  log << " - NEW";
                  prev_x_ = msg->point.x;
                  prev_y_ = msg->point.y;
                  prev_z_ = msg->point.z;
                }
                else{
                  is_object_detected_ = false;
                  log << " - ALREADY DETECTED";
                }
                IauvGirona1000Survey::printIfFromLastPrintHavePassedSomeSeconds(log.str(), 1.0, prev_printed_msg_,last_print_time_);
            }
            
            else
            {
                is_object_detected_ = false;
            }
        }
    }

BT::NodeStatus isPathClear::tick() 
    {
        // set survey type
        // TODO: implement logic to enter this block only if path changed
        std::string survey_type_str;
        // FIXME: it doesn't look ike it changes dynamically. maybe it must be subscribed
        if (getInput<std::string>("survey_type", survey_type_str)) {
        if (survey_type_str == "scan") {
            survey_type_ = IauvGirona1000Survey::SCAN;
            //  ROS_INFO("[ isPathClear ]: Survey type set to SCAN");
        } else if (survey_type_str == "circular") {
            survey_type_ = IauvGirona1000Survey::CIRCULAR;
            //  ROS_INFO("[ isPathClear ]: Survey type set to CIRCULAR");
        } else {
            throw BT::RuntimeError("[ isPathClear ]: Invalid survey type");
        }
    } 
        
        return (is_object_detected_) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        // auto val = config().blackboard->get<bool>(port_name_);
        // return (val) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

