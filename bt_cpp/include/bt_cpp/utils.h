#pragma once

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