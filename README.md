# BT ACE
Repo for automated mission of Girona 1000 with robotic arm to scan and collect data of an area, communicate through luma and interact with objects found.

## What's inside
- `bt_cpp` package
- `pcl_geometric_primitives_detector` package

## Installation
- Once you install ROS Noetic, create a workspace (eg. `~/catkin_ws`) following ROS instructions for ws creation (either `catkin_make` or -the one I used - `catkin init`)

### IAUV Girona 1000 use case
This branch is built to work with [iauv_demo](https://github.com/GitSRealpe/iauv_demo/): git clone this repo in the same `~/catkin_ws/src` and follow the instructions provided by the repo. I used this [commit](https://github.com/GitSRealpe/iauv_demo/commit/4848c8fe560a5b6e492adde968dbb2573105e5b6). PTAL at `other_repo_modifications`

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
# launch simulation w/ Stonefish, RVIZ, service to plan the path and action service to follow the path
roslaunch bt_cpp scenario.launch
```

T2
```bash
# launch node to make available the planner services
roslaunch pcl_geometric_primitives_detector perception.launch
```

T3
```bash
# Launch the BT Manager
rosrun bt_cpp bt_manager
```

T4
```bash
# to keep separate for debugging - to be included in perception.launch
rosrun pcl_geometric_primitives_detector pcl_geometric_primitives_detector
```

perception.launch
rosservice call /validate_mission
populate stack for mission(scan + circle around cube)
execute stack
rosrun pcl_geometric_primitives_detector pcl_geometric_primitives_detector
go to luma -> "Received PointCloud data."
rosservice call /cluster "{}"
take labeledObjInfo (published by pcl_geometric_primitives_detector)
getAction (labeledObjInfo)
populate bt stack with doable actions(bts)
execute stack

## TODO

### PCL
- debug affordanceGraph /validate_mission (now there is an hacky-fix):
I changed `affordanceGraph.py` from:
```python
                 if len(parts) == 3 and parts[1] == "do":  # Example: "AUV do Survey"
                    subject = parts[0]

                    action = parts[2]
                    target = None
 

                elif len(parts) == 4 and parts[1] == "look" and parts[2] == "at":  # Example: "AUV look at Sphere"
                    subject = parts[0]
                    action = "observe"  # Map "look at" to "observe" as per affordance graph terminology
                    target = parts[3]
                    rospy.set_param('/target', target)
```

to:

```python
                 if parts[1] == "do":  # Example: "AUV do Survey"
                    subject = parts[0]
                    action = "allows"
                    target = parts[2]
                    action = parts[2]
                    target = None
 
                elif parts[1] == "look_at":  # Example: "AUV look_at Sphere"
                    subject = parts[0]
                    action = "observe"  # Map "look at" to "observe" as per affordance graph terminology
                    target = parts[2]
                    rospy.set_param('/target', target)

```

and mission_file.txt from:
```
AUV do Survey
AUV look at Sphere
```
to:

```
AUV do Survey
AUV look_at Sphere
```

One request here: please keep the format as 3 words (`subject` `action` `target`) and, if you need a composed word, use the underscore (eg. `look at` -> `look_at`)

- debug `pcl_geometric_primitives_detector.cpp` (it gives `Segmentation Fault` in `detectPlane` inside the while. It's either when it has to recheck the while condition `while (!remaining_cloud->points.empty())` or when it executes `extract.filter(*remaining_cloud);`)

### BT
- subscriber to labelObjInfo
- getAction
- assemble info (type of actions and coord relative objects) to communicate target position, prioritize bt and populate stack
- verify third part of mission