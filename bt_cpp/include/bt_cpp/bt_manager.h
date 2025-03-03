#include "behaviortree_cpp/bt_factory.h"
#include "bt_cpp/object_sorter.h"
#include <deque>
#include <string>
#include <memory>
#include <ros/ros.h>
#include <functional>

struct SortedObjectInfo {
    std::string name;
    std::vector<std::string> actions;
    std::string start;
    std::string goal;
};

class BTManager {
public:
    BTManager(const std::string& xml_path, ros::NodeHandle& nh, BT::BehaviorTreeFactory& factory)
        : xml_path_(xml_path), nh_(nh), factory_(factory), status_(BT::NodeStatus::IDLE), offset_from_object_(2.0)
    {}

    void populateStack(const std::vector<std::string>& bt_exec_ids);
    void executeStack();
    void executeDynamicStack();
    void populateSortedObjectsStack(const std::vector<SortedObjectInfo>& sorted_objects);
    void populateSortedObjectsStackFromSorter(const std::vector<LabeledObject>& sorted_objects, std::string start);
    void printSortedObjectsStack() const;
    std::string adjustXCoordinate(const std::string& point, double offset);
    void executeAction(const std::string& xml_tree, const std::string& start, const std::string& goal, const std::string& name);
    void executeStackWFun(std::function<void()> fun_executed);
    void BtRunWFun(BT::NodeStatus& status, std::unique_ptr<BT::Tree>& tree, std::function<void()> fun_executed);

private:

    std::string constructPath(const std::string& xml_tree);

    void BtRun(BT::NodeStatus& status, std::unique_ptr<BT::Tree>& tree);

    ros::NodeHandle& nh_;
    BT::NodeStatus status_;
    BT::BehaviorTreeFactory& factory_;
    std::deque<std::string> stack_;
    std::string xml_path_;
    std::deque<SortedObjectInfo> sorted_objects_stack_; 
    double offset_from_object_; // distance to kep from object while computing actions
};
