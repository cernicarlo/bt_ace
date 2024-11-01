#include "behaviortree_cpp/bt_factory.h"
#include <deque>
#include <string>
#include <memory>
#include <ros/ros.h>

class BTManager {
public:
    BTManager(const std::string& xml_path, ros::NodeHandle& nh, BT::BehaviorTreeFactory& factory)
        : xml_path_(xml_path), nh_(nh), factory_(factory), status_(BT::NodeStatus::IDLE)
    {}

    void populateStack(const std::vector<std::string>& bt_exec_ids);
    void executeStack();

private:

    std::string constructPath(const std::string& xml_tree);

    void BtRun(BT::NodeStatus& status, std::unique_ptr<BT::Tree>& tree);

    ros::NodeHandle& nh_;
    BT::NodeStatus status_;
    BT::BehaviorTreeFactory& factory_;
    std::deque<std::string> stack_;
    std::string xml_path_;
};
