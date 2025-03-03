cmake_minimum_required(VERSION 3.5)

project(bt_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS 
  behaviortree_cpp
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  girona_utils
  iauv_motion_planner
  dual_gironas
  )


add_message_files(
  FILES
  LabeledObjInfo.msg
)


add_service_files(
  FILES
  Cluster.srv
  GetActions.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  behaviortree_cpp 
  roscpp 
  actionlib 
  actionlib_msgs
  message_runtime
  girona_utils
  iauv_motion_planner
  dual_gironas
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

# NODES 
add_library(nodes STATIC
    nodes/bt_manager.cpp
    nodes/dock_interact_node.cpp
    nodes/follow_path_node.cpp
    nodes/path_request_node.cpp
    nodes/is_path_clear_node.cpp
    nodes/primitive_detection_client.cpp
    nodes/object_sorter.cpp
    nodes/mission_utils.cpp
    )

target_include_directories(nodes PRIVATE 
    ${catkin_INCLUDE_DIRS} 
    )
target_link_libraries(nodes
    ${catkin_LIBRARIES} 
    )

# target main
find_package(catkin REQUIRED COMPONENTS behaviortree_cpp)
catkin_package(CATKIN_DEPENDS behaviortree_cpp )

add_executable(scan_path src/scan_path.cpp)
target_include_directories(scan_path PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(scan_path ${catkin_LIBRARIES} nodes)

add_executable(bt_manager src/exec_bt_manager.cpp)
target_include_directories(bt_manager PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(bt_manager ${catkin_LIBRARIES} nodes)

add_executable(llm_bt_manager src/exec_llm_bt_manager.cpp)
target_include_directories(llm_bt_manager PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(llm_bt_manager ${catkin_LIBRARIES} nodes)

add_executable(llm_bt_manager_a src/exec_llm_bt_manager_a.cpp)
target_include_directories(llm_bt_manager_a PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(llm_bt_manager_a ${catkin_LIBRARIES} nodes)

add_executable(llm_bt_manager_b src/exec_llm_bt_manager_b.cpp)
target_include_directories(llm_bt_manager_b PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(llm_bt_manager_b ${catkin_LIBRARIES} nodes)

# Install the targets of bt_cpp
install(TARGETS scan_path bt_manager
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



catkin_install_python(PROGRAMS
  scripts/detect_object.py
  scripts/odometry_girona.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)