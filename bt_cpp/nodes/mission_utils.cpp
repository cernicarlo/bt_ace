// mission_utils.cpp
#include "bt_cpp/mission_utils.h"
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <fstream>
#include <sstream>

bool validateMission(ros::NodeHandle& nh) {
    ros::ServiceClient vm_client = nh.serviceClient<std_srvs::Trigger>("/validate_mission");
    std_srvs::Trigger trigger_srv;

    if (vm_client.call(trigger_srv)) {
        if (trigger_srv.response.success) {
            ROS_INFO("Mission validation successful: %s", trigger_srv.response.message.c_str());
            return true;
        } else {
            ROS_ERROR("Mission validation failed: %s", trigger_srv.response.message.c_str());
            return false;
        }
    } else {
        ROS_ERROR("Failed to call service /validate_mission");
        return false;
    }
}

bool createMissionStack(ros::NodeHandle& nh, std::vector<std::string>& action_targets) {
    std::string mission_file_path;
    if (!nh.getParam("/mission_file", mission_file_path)) {
        ROS_ERROR("Failed to get /mission_file parameter");
        return false;
    }

    std::ifstream mission_file(mission_file_path);
    if (!mission_file.is_open()) {
        ROS_ERROR("Failed to open mission file: %s", mission_file_path.c_str());
        return false;
    }

    std::string line;
    while (std::getline(mission_file, line)) {
        std::istringstream iss(line);
        std::string subject, action, target;

        if (!(iss >> subject >> action >> target)) {
            ROS_WARN("Skipping malformed line: %s", line.c_str());
            continue;
        }

        if (subject == "AUV") {
            std::string action_target = action + "_" + target;
            action_targets.push_back(action_target);
            ROS_INFO("Created action_target: %s", action_target.c_str());
        }
    }

    mission_file.close();
    return true;
}

void spinUntilCondition(const std::function<bool()>& condition) {
    while (ros::ok() && !condition()) {
        ros::spinOnce();  // Process callbacks
        ros::Duration(0.1).sleep();
    }
}