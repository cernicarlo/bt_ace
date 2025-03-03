// mission_utils.h
#pragma once

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <vector>
#include <string>

bool validateMission(ros::NodeHandle& nh);
bool createMissionStack(ros::NodeHandle& nh, std::vector<std::string>& action_targets);
void spinUntilCondition(const std::function<bool()>& condition);

