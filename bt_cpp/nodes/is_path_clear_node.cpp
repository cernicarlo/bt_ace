
#include "bt_cpp/is_path_clear_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "iauv_motion_planner/GetPath.h"
#include "iauv_motion_planner/GetPathRequest.h"
#include "iauv_motion_planner/PlannerParam.h"


namespace IauvGirona1000Survey {


void isPathClear::printIfFromLastPrintHavePassedSomeSeconds(const std::string& msg, double seconds)
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time_);
        if (elapsed.count() >= seconds * 1000)
        {
            ROS_INFO_STREAM(msg);
            last_print_time_ = now;
        }
    }

void isPathClear::objectPoseCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        if (survey_type_ == SCAN)
        {
            std::stringstream log;
            log << "SCAN: ";
            if (!std::isnan(msg->point.x))
            {
                is_object_detected_ = true;
                log << "Received object position - x: " << msg->point.x
                    << ", y: " << msg->point.y
                    << ", z: " << msg->point.z;
                printIfFromLastPrintHavePassedSomeSeconds(log.str(), 1.0);
            }
            else
            {
                is_object_detected_ = false;
            }
        }
    }

BT::NodeStatus isPathClear::tick() 
    {
        if (!is_object_detected_)
        {
            // ROS_INFO("No object detected.");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_INFO("Object detected.");
            return BT::NodeStatus::FAILURE;
        }
    }

}