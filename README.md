# BT ACE
Repo for automated mission of Girona 1000 with robotic arm to scan and collect data of an area, communicate through luma and interact with objects found.

## What's inside
- `bt_cpp` package
- `pcl_geometric_primitives_detector` package

## Installation
- Once you install ROS Noetic, create a workspace (eg. `~/catkin_ws`) following ROS instructions for ws creation (either `catkin_make` or -the one I used - `catkin init`)

### IAUV Girona 1000 use case
This branch is built to work with [iauv_demo](https://github.com/GitSRealpe/iauv_demo/): git clone this repo in the same `~/catkin_ws/src` and follow the instructions provided by the repo. I used this [commit](https://github.com/GitSRealpe/iauv_demo/commit/4f2df70af35bb16ef5fda9cbb9a80e60c6ebf021). For this repo, the only modification made was in `iauv_demo/iauv_description/launch/core/girona1000_base.launch.core L43-45`
```xml
    <!-- <node name="captain" pkg="cola2_control" type="captain_node" output="screen">
        <param name="vehicle_config_launch_mission_package" value="cola2_$(arg robot_name)"/>
    </node> -->

```

I also cloned and built these public repo (some of them may be not relevant for this project):
- [blueprintlab_reachbravo7_manipulator_description](https://bitbucket.org/udg_cirs/blueprintlab_reachbravo7_manipulator_description.git)
- [cola2_core](https://bitbucket.org/iquarobotics/cola2_core.git)
- [cola2_girona1000](https://bitbucket.org/udg_cirs/cola2_girona1000.git)
- [cola2_lib](https://bitbucket.org/iquarobotics/cola2_lib.git)
- [cola2_lib_ros](https://bitbucket.org/iquarobotics/cola2_lib_ros.git)
- [cola2_msgs](https://bitbucket.org/iquarobotics/cola2_msgs.git)
- [cola2_stonefish](https://bitbucket.org/iquarobotics/cola2_stonefish.git)
- [girona1000_description](https://bitbucket.org/udg_cirs/girona1000_description.git)
- [girona_utils](https://github.com/GitSRealpe/girona_utils.git)
- [stonefish](https://github.com/Michele1996/stonefish/tree/vlc_dev)
- [stonefish_ros](https://github.com/Michele1996/stonefish_ros)
- [interactive markers](https://github.com/ros-visualization/interactive_markers)

Please take a look at the folder `other_repo_modification` to substitute the relative file in the other repo involved in this use case.


### BT CPP
- clone [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) (last time I checked, it was using this [commit](https://github.com/BehaviorTree/BehaviorTree.CPP/commit/1fcb624d4d7b9d1f357378b20ff19bdcc3853cea)) in `src` (eg. `~/catkin_ws/src`)
- clone this repo in the same `src` folder (eg. `~/catkin_ws/src`)
- modify `bt_ace/bt_cpp/src/scan_path.cpp` the variable `abs_repo_path` with the path where is located your local `bt_ace/bt_cpp/xml` file  and rebuild<!-- TODO: assign a dynamic variable -->
- build the ws (eg. execute `catkin build` in `~/catkin_ws`)
- `source ~/catkin_ws/devel/setup.bash`

### PCL geometric primitives detecor
* before starting install [PCL 1.14](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.14.1) manually. [Here](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html) more info.

```bash
 wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.14.1.tar.gz
 tar -xzf pcl-1.14.1.tar.gz
 cd pcl-pcl-1.14.1
 mkdir build
 cd build
 cmake ..

 # the number depends on how many jobs you want to use
 make -j10
 sudo make -j10 install
```
* install groq (`pip install groq`)
* install graphviz (`pip install graphviz`)
* install open3d (`pip install open3d`)
* install yaml (`pip install yaml`)
* install tkinter (`pip install tkinter`)



## Use Case
WIP for girona1000 use case: the AUV goes to do the survey, it suddenly find an object it was not planned to find, it searches among its possibilities and it detects the bt that can make a turn around the object

T1
```bash
# launch simulation w/ Stonefish and RVIZ
roslaunch iauv_description single_robot.launch
```

T2
```bash
# launch node to make available the planner services
roslaunch iauv_motion_planner planner_node.launch
```

T3
```bash
# enable the pose controller for the auv
roslaunch girona_utils auv_pose_controller.launch robot:=girona1000
```

<!-- T4
```bash
# start the node that will detect the object (at this point, it uses priviledged info - TODO: make it real)
rosrun bt_cpp detect_object.py
``` -->


T5
```bash
rosrun bt_cpp scan_path
```


perception.launch
(check mission file?)
scan + circle around cube
go to luma -> "Received PointCloud data."
rosservice call /cluster "{}"

## TODO

### BT
- adjust condition isPathClear
- test whole path
- write nodes for welcoming clustering scripts
- write nodes to interact with outside
