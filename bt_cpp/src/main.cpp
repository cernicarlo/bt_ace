#include "behaviortree_cpp/bt_factory.h"
#include "bt_cpp/inspect_node.h"
#include "bt_cpp/path_request_node.h"
#include "bt_cpp/is_path_clear_node.h"
#include <fstream>
#include <iostream>
#include <cmath>
// #include "behaviortree_cpp/sample_nodes/dummy_nodes.h"

using namespace BT;

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

BT::NodeStatus SaySomethingSimple(BT::TreeNode& self)
{
  auto msg = self.getInput<std::string>("message");
  if(!msg)
  {
    throw BT::RuntimeError("missing required input [message]: ", msg.error());
  }

  std::cout << "Robot says: " << msg.value() << std::endl;
  return BT::NodeStatus::SUCCESS;
}

// TODO: import it in the relative header/cpp file
class isPathClear : public BT::ConditionNode
{
private:
  std::string port_name_;
  ros::Subscriber object_pose_sub_;
  bool is_object_detected_;
  IauvGirona1000Survey::SurveyType survey_type_;
  std::chrono::steady_clock::time_point last_print_time_;
  ros::NodeHandle nh_;
  // TODO: these work only for this case, otherwise, we need a vector of points and some tolerance
  double prev_x_ , prev_y_, prev_z_;

public:
  isPathClear(const std::string& name, const BT::NodeConfig& config,
                  std::string port_name, ros::NodeHandle nh)
    : BT::ConditionNode(name, config), port_name_(port_name), is_object_detected_(false), nh_(nh),
    prev_x_(NAN), prev_y_(NAN), prev_z_(NAN)
  {
    object_pose_sub_ = nh_.subscribe("/object_pose", 10, &isPathClear::objectPoseCallback, this);
  }
  
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("survey_type") };
  };

  // Callback function for the subscriber
  void objectPoseCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
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
                printIfFromLastPrintHavePassedSomeSeconds(log.str(), 1.0);
            }
            
            else
            {
                is_object_detected_ = false;
            }
        }
    }

    // Helper function to manage log output frequency
    void printIfFromLastPrintHavePassedSomeSeconds(const std::string& msg, double seconds)
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time_);
        if (elapsed.count() >= seconds * 1000)
        {
            ROS_INFO_STREAM(msg);
            last_print_time_ = now;
        }
    }
  
  BT::NodeStatus tick() override
  {
    ros::spinOnce();

    // set survey type
    // TODO: implement logic to enter this block only if path changed
    std::string survey_type_str;
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
    
    return (is_object_detected_) ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
    // auto val = config().blackboard->get<bool>(port_name_);
    // return (val) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bt_cpp_node");

  std::string abs_repo_path = "/home/ros/catkin_ws/src/";
  std::string relative_xml_fold_path = "bt_ace/bt_cpp/bt_xml/";
  std::string xml_tree = "iauv_girona1000_survey_scan.xml";
  // BehaviorTreeFactory factory;

  std::unique_ptr<BT::BehaviorTreeFactory> factory;
  std::string path;
  ros::NodeHandle nh;
  // path = "/home/ros/catkin_ws/src/bt_ace/bt_cpp/bt_xml/iauv_girona1000_survey_circular_true.xml";
  path = abs_repo_path + relative_xml_fold_path + xml_tree;
  
  factory = std::make_unique<BT::BehaviorTreeFactory>();
  
  // iauv girona1000 survey
  registerCustomNode<IauvGirona1000Survey::PathRequest>(
      *factory, "PathRequest", nh);


  // registerCustomNode<IauvGirona1000Survey::isPathClear>(
  //     *factory, "isPathClear", nh);
  
  factory->registerNodeType<isPathClear>("isPathClear", "is_path_clear", nh);
  // factory->registerNodeType<IauvGirona1000Survey::isPathClear>("isPathClear", "is_path_clear");
  // registerSimpleCondition(
  //         "isPathClear",
  //         std::bind(IauvGirona1000Survey::Inspect::isPathClear));

  PortsList say_something_ports = { InputPort<std::string>("message") };
  factory->registerSimpleAction("SaySomething2", SaySomethingSimple, say_something_ports);

  // factory->registerNodeType<DummyNodes::SaySomething>("SaySomething");
  registerCustomNode<IauvGirona1000Survey::Inspect>(
      *factory, "Inspect", nh);
   
  
  // factory.registerNodeType<SaySomething>("SaySomething");
  // factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");

  // Parse the XML file and create a tree_ from it
  factory->registerBehaviorTreeFromFile(path);
  std::unique_ptr<BT::Tree> tree;
  std::string bt_id = "main_bt";
  tree = std::make_unique<BT::Tree>(
       factory->createTree(bt_id));
  
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  while (status != BT::NodeStatus::SUCCESS &&
          status != BT::NodeStatus::FAILURE && ros::ok()) {
    status = tree->tickOnce();      // Tick the tree once
    ros::spinOnce();       // Process ROS callbacks
    // loop_rate.sleep();     // Sleep to maintain loop rate
  }

  return 0;
}
