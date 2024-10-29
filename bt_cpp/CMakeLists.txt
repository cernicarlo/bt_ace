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
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

# NODES 
add_library(nodes STATIC
    nodes/inspect_node.cpp
    nodes/path_request_node.cpp
    nodes/is_path_clear_node.cpp
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

add_executable(bt_cpp src/main.cpp)
target_include_directories(bt_cpp PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(bt_cpp ${catkin_LIBRARIES} nodes)

# Install the target bt_cpp
install(TARGETS bt_cpp
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS bt_cpp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


catkin_install_python(PROGRAMS
  scripts/detect_object.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)