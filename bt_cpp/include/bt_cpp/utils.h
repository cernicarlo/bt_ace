#ifndef UTILS_H
#define UTILS_H

#include "behaviortree_cpp/bt_factory.h"
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <sstream>

namespace IauvGirona1000Survey {


enum SurveyType
{
    SCAN = 1,     // 1 represents scan
    CIRCULAR = 2,  // 2 represents circular
    SIMPLE = 3, // 3 represent simple
};

// Custom type
struct Pose3D
{
    double x, y, z;
};

inline std::string pointToString(const geometry_msgs::Point& point) {
    std::ostringstream oss;
    oss << point.x << ";" << point.y << ";" << point.z;
    return oss.str();
}

inline bool hasEnoughTimePassed(double seconds, std::chrono::steady_clock::time_point &last_print_time_) {
   auto now = std::chrono::steady_clock::now();
   auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
       now - last_print_time_);
   return elapsed.count() >=
          seconds * 1000;  // Convert seconds to milliseconds and compare
}

inline void printIfFromLastPrintHavePassedSomeSeconds(
    const std::string& msg, double seconds, std::string &prev_printed_msg_, std::chrono::steady_clock::time_point &last_print_time_) {
   if (msg != prev_printed_msg_) {
      std::cout << msg << std::endl;
      prev_printed_msg_ = msg;
      return;
   }

   if (hasEnoughTimePassed(seconds, last_print_time_)) {
      std::cout << msg << std::endl;
      last_print_time_ =
          std::chrono::steady_clock::now();  // Update the last print time
   }
}

}

template <typename NodeType>
void registerCustomNode(BT::BehaviorTreeFactory& factory,
                        const std::string& registration_id,
                        ros::NodeHandle& nh) {
   // Use the registration_id directly as the type identifier
   BT::NodeBuilder builder = [&nh](
                                 const std::string& name,
                                 const BT::NodeConfig& config) {
      return std::make_unique<NodeType>(nh, name, config);
   };

   // Assuming you directly use the registration_id as the manifest type
   factory.registerBuilder<NodeType>(registration_id, builder);
}

#endif  // UTILS_H