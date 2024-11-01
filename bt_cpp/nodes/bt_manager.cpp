#include "bt_cpp/bt_manager.h"
#include "behaviortree_cpp/bt_factory.h"
#include <deque>
#include <string>
#include <memory>
#include <ros/ros.h>

void BTManager::populateStack(const std::vector<std::string>& bt_exec_ids) {
        for (const auto& bt_id : bt_exec_ids) {
            stack_.push_back(bt_id);
        }
    }

void BTManager::executeStack() {
    while (!stack_.empty() && ros::ok()) {
        std::string xml_tree = stack_.front();
        stack_.pop_front();

        std::string path = constructPath(xml_tree);
        auto tree = std::make_unique<BT::Tree>(factory_.createTreeFromFile(path));

        BtRun(status_, tree);
        status_ = BT::NodeStatus::IDLE;
    }
}


std::string BTManager::constructPath(const std::string& xml_tree) {
    return xml_path_ + xml_tree + ".xml";
}

void BTManager::BtRun(BT::NodeStatus& status, std::unique_ptr<BT::Tree>& tree) {
    while (status != BT::NodeStatus::SUCCESS &&
            status != BT::NodeStatus::FAILURE &&
            ros::ok()) {
        status = tree->tickOnce();
        ros::spinOnce();
    }
}

