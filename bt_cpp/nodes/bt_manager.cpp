#include "bt_cpp/bt_manager.h"
#include "behaviortree_cpp/bt_factory.h"
#include "bt_cpp/object_sorter.h"
#include "bt_cpp/utils.h"

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

void BTManager::populateSortedObjectsStack(const std::vector<SortedObjectInfo>& sorted_objects) {
    for (const auto& obj : sorted_objects) {
        sorted_objects_stack_.push_back(obj);
    }
}

void BTManager::printSortedObjectsStack() const {
    if (sorted_objects_stack_.empty()) {
        ROS_INFO("The sorted objects stack is empty.");
        return;
    }

    ROS_INFO("\nContents of the sorted objects stack:");
    for (const auto& obj : sorted_objects_stack_) {
        ROS_INFO("Name: %s", obj.name.c_str());
        ROS_INFO("Actions:");
        for (const auto& action : obj.actions) {
            ROS_INFO(" - %s", action.c_str());
        }
        ROS_INFO("Start: %s", obj.start.c_str());
        ROS_INFO("Goal: %s", obj.goal.c_str());
        ROS_INFO("---------------------------");
    }
}

void BTManager::populateSortedObjectsStackFromSorter(const std::vector<LabeledObject>& sorted_objects, std::string start) {
    std::vector<SortedObjectInfo> sorted_object_info_list;

    for (size_t i = 0; i < sorted_objects.size(); ++i) {
        SortedObjectInfo info;
        info.name = sorted_objects[i].label;
        
        info.actions = sorted_objects[i].info.actions;
        
        
        // Set goal using the surface_point of the current object
        info.goal = IauvGirona1000Survey::pointToString(sorted_objects[i].info.surface_point);
        
        // For the first object, set start to a default value or initial starting point
        info.start = (i == 0) ? start : sorted_object_info_list.back().goal;

        sorted_object_info_list.push_back(info);
    }

    // Now populate the stack with the transformed list
    populateSortedObjectsStack(sorted_object_info_list);
}



void BTManager::executeDynamicStack() {
    while (!sorted_objects_stack_.empty() && ros::ok()) {
        auto& current_object = sorted_objects_stack_.front();
        std::string name = current_object.name;

        // Initial goal adjustment for the "goto" action
        std::string start = current_object.start;
        std::string goal = adjustXCoordinate(current_object.goal, offset_from_object_);  // Modified goal for "goto"

        // Execute "goto" action first
        std::string goto_tree = "goto";
        executeAction(goto_tree, start, goal, name);

        // Set up the unmodified goal for subsequent actions
        goal = current_object.goal;

        // Iterate over all actions
        for (const auto& action : current_object.actions) {
            // Start becomes the modified goal from "goto" (x adjusted by -2)
            start = adjustXCoordinate(goal, offset_from_object_);

            // Execute the action tree with adjusted start and original goal
            executeAction(action, start, goal, name);
        }

        // Remove the processed object from the stack
        sorted_objects_stack_.pop_front();
    }
}

// Helper function to adjust the X-coordinate of a point string
std::string BTManager::adjustXCoordinate(const std::string& point, double offset) {
    // Parse the string (format assumed to be "x;y;z")
    std::istringstream ss(point);
    double x, y, z;
    char delimiter;
    ss >> x >> delimiter >> y >> delimiter >> z;

    // Adjust the x-coordinate
    x += offset;
    y += offset;

    // Reconstruct the point string
    std::ostringstream adjusted_point;
    adjusted_point << x << ";" << y << ";" << z;
    return adjusted_point.str();
}

// Helper function to execute a tree with specified start and goal
void BTManager::executeAction(const std::string& xml_tree, const std::string& start, const std::string& goal, const std::string& name) {
    std::cout << "[BT dynamic manager] : executing tree " << xml_tree << " for object "<< name <<std::endl;

    // Construct tree from XML file
    std::string path = constructPath(xml_tree);
    auto tree = std::make_unique<BT::Tree>(factory_.createTreeFromFile(path));

    if ((xml_tree == "observe") || (xml_tree == "goto")){
        // Set blackboard variables for start and goal positions
        auto blackboard = tree->rootBlackboard();
        blackboard->set("start", start);
        blackboard->set("goal", goal);
    }
    
    // Run the behavior tree
    BtRun(status_, tree);
    status_ = BT::NodeStatus::IDLE;
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



void BTManager::executeStackWFun(std::function<void()> fun_executed) {
    while (!stack_.empty() && ros::ok()) {
        std::string xml_tree = stack_.front();
        stack_.pop_front();

        std::string path = constructPath(xml_tree);
        auto tree = std::make_unique<BT::Tree>(factory_.createTreeFromFile(path));

        BtRunWFun(status_, tree, fun_executed);
        status_ = BT::NodeStatus::IDLE;
    }
}

void BTManager::BtRunWFun(BT::NodeStatus& status, std::unique_ptr<BT::Tree>& tree, std::function<void()> fun_executed) {
    while (status != BT::NodeStatus::SUCCESS &&
            status != BT::NodeStatus::FAILURE &&
            ros::ok()) {
        status = tree->tickOnce();
        ros::spinOnce();

        // Execute the additional function passed as argument
        fun_executed();
    }
}
