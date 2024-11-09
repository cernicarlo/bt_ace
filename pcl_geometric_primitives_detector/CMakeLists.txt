cmake_minimum_required(VERSION 3.0.2)
project(pcl_geometric_primitives_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  ros_numpy
)

find_package(PCL 1.14 REQUIRED COMPONENTS common filters segmentation visualization)


add_message_files(
        FILES
        ClusterObjInfo.msg
        LabeledObjInfo.msg
        SimulationInfo.msg
      )

add_service_files(
  FILES
  Cluster.srv
  ClusterResponse.srv
  PlanGoal.srv
  AddObject.srv
  AddAffordance.srv
  ExportGraph.srv
  CreateGUI.srv
  DisplayGraph.srv
  CheckRelation.srv
  GetActions.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

# Declare the catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pcl_geometric_primitives_detector
  CATKIN_DEPENDS sensor_msgs pcl_ros pcl_conversions ros_numpy
)

# Build the C++ executable
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcl_geometric_primitives_detector src/pcl_geometric_primitives_detector.cpp)

# Link against required ROS 1 and PCL libraries
target_link_libraries(pcl_geometric_primitives_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)



# Install the executable
install(TARGETS pcl_geometric_primitives_detector
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Add definitions from PCL if necessary
add_definitions(${PCL_DEFINITIONS})


catkin_install_python(PROGRAMS
  scripts/affordanceGraphN.py
  scripts/marker_llm.py
  scripts/object_sensing_ACE.py
  scripts/operator_label_2_obj_label.py
  scripts/pcd_compressor.py
  scripts/label_obj_pub_mock.py
  scripts/get_actions_mock.py
  scripts/store_pointcloud.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Optionally add testing dependencies
# if(CATKIN_ENABLE_TESTING)
#   find_package(roscpp_tutorials REQUIRED)
#   catkin_add_gtest(${PROJECT_NAME}_test test/pcl_geometric_primitives_detector_test.cpp)
#   target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})
# endif()

