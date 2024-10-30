# bt_policy
This project is inspired by [btcpp_sample](https://github.com/BehaviorTree/btcpp_sample) and, hereby, compatible with version 4.X of [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) (that you must download and build for this repo to work together with ROS noetic - ).

The **CMakeLists.txt** and **package.xml** files are compatible with both ROS and ROS2.

## Python Requirements
For the perception and knowledge part, you need to have: 
```
groq
open3d
tkinter
yaml
graphviz
```
Then the [interactive markers](http://wiki.ros.org/interactive_markers)

## Installation
- Once you install ROS Noetic, create a workspace (eg. `~/catkin_ws`) following ROS instructions for ws creation (either `catkin_make` or -the one I used - `catkin init`)
- clone [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) (last time I checked, it was using this [commit](https://github.com/BehaviorTree/BehaviorTree.CPP/commit/1fcb624d4d7b9d1f357378b20ff19bdcc3853cea)) in `src` (eg. `~/catkin_ws/src`)
- clone this repo in the same `src` folder (eg. `~/catkin_ws/src`)
- modify `bt_policy/include/bt_policy/utils.h L14` the variable `path` with the path where is located your local `bt_policy/xml` file  and rebuild<!-- TODO: assign a dynamic variable -->
- build the ws (eg. execute `catkin build` in `~/catkin_ws`)
- `source ~/catkin_ws/devel/setup.bash`

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
- [stonefish_ros] (https://github.com/patrykcieslak/stonefish.git)

Please take a look to the folder `other_repo_modification` to substitute the relative file in the other repo involved in this use case.


## For users
- add/remove the BT in `bt_cpp/bt_xml` file following the conventions

## For developers
* create the node in the `bt_cpp/nodes` folder
* update src/main.cpp with new nodes


## Use Case AUV:
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

T4
```bash
# start the node that will detect the object (at this point, it uses priviledged info - TODO: make it real)
rosrun bt_cpp detect_object.py
```


T5
```bash
# bt client
```

## Perception and Knowledge Representation Scripts
The launch file perception.launch in the launch folder runs the following scripts:
```bash
affordanceGraphN.py
marker_llm.py
object_sensing_ACE.py
pcd_compressor.py
```
### PCD Compressor
The script checks if the TFs of the two vlc are seeing each other and if the distance is short enough (depending on the water quality). In the positive case, it gets the PointCloud2 message from "/girona1000/depth_cam/pointcloud", compresses and publishes it to "/luma_station/compressed_cloud".

### ObjectSensing
This script does clustering of the PointCloud2 received from the topic "/luma_station/compressed_cloud".
It uses the silhouette score to compute the best number of clusters and then applies K-Means Cluster to compute them. 
It then computes for each cluster its centroid and a surface point and publishes it as [ClusterObjInfo](https://github.com/cernicarlo/bt_ace/blob/master/pcl_geometric_primitives_detector/msg/ClusterObjInfo.msg) message to "/clustered_point_cloud"

### MarkerLLM
The scripts use Groq as LLM
```bash
client = Groq(

    api_key='',

)
```
You need to create an api key. Check https://console.groq.com/keys

The node subscribes to the "/labeled_cloud" topic and gets [LabeledObjInfo](https://github.com/cernicarlo/bt_ace/blob/master/pcl_geometric_primitives_detector/msg/LabeledObjInfo.msg) msg which contains the label, the PointCloud2, its centroid and a surface point
