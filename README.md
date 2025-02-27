# BT ACE
Repository for automated mission of Girona 1000 with robotic arm to scan and collect data of an area, communicate through luma and interact with objects found.


# Table of Contents
1. [What's Inside](#whats-inside)
2. [Installation](#installation)
   - [PCL Geometric Primitives Detector](#pcl-geometric-primitives-detector)
   - [IAUV Girona 1000 Use Case](#iauv-girona-1000-use-case)
3. [Use Case](#use-case)
   - [Mission Flow](#mission-flow)
4. [Troubleshooting](#troubleshooting)


## What's inside
<!-- Description packages? -->
- `bt_cpp` package
- `pcl_geometric_primitives_detector` package

## Installation
- Install this stonefish version: https://github.com/Michele1996/stonefish/tree/vlc_dev (documentation at: https://stonefish.readthedocs.io/)
- Once you install ROS Noetic, create a workspace (eg. `~/catkin_ws`) following ROS instructions for ws creation (either `catkin_make` or `catkin init`)

```bash
sudo apt install git python3-pip python3-vcstool python3-tk python3-opencv libusb-1.0-0-dev libfcl-dev
```

### PCL geometric primitives detecor
* before starting, install [PCL 1.14](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.14.1) manually. [Here](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html) more info.

```bash
 wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.14.1.tar.gz
 tar -xzf pcl-1.14.1.tar.gz
 cd pcl-pcl-1.14.1
 mkdir build
 cd build
 cmake ..

 # the number (10 in this case) depends on how many jobs you want to use
 make -j10
 sudo make -j10 install
```

<!-- TODO: put this in a requirement.txt file and add the instruction to install them with the version we used -->
* install groq (`pip install groq`)
* install graphviz (`pip install graphviz`)
* install open3d (`pip install open3d`)
* install yaml (`pip install pyyaml`)
* install openai (`pip install openai`)
* one command line: `pip install groq graphviz open3d openai pyyaml ruamel.yaml statemachine python-statemachine pydot`

### IAUV Girona 1000 use case
1. To ensure compatibility, clone the dependencies at the specified versions. Navigate to the `src` directory of your Catkin workspace and clone all the needed repositories:

```bash
cd ~/catkin_ws/src
git clone https://github.com/cernicarlo/bt_ace.git
vcs import < ~/catkin_ws/src/bt_ace/repos.vc
```
2. Follow the instructions to install stonefish at `~/catkin_ws/src/stonefish/README.md` (previously downloaded with `vcs`) 
3. Install dependencies
<!-- TODO: reseolve with this (to be tested in a fresh environment):
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```
 -->
```bash
sudo apt install ros-noetic-interactive-markers ros-noetic-ros-numpy ros-noetic-joint-trajectory-controller ros-noetic-fcl ros-noetic-velocity-controllers ros-noetic-joint-state-controller ros-noetic-joint-state-publisher-gui ros-noetic-octomap-server ros-noetic-depth-image-proc ros-noetic-octomap-rviz-plugins lm-sensors
```

<!-- This branch is built to work with [iauv_demo](https://github.com/GitSRealpe/iauv_demo/): git clone this repo in the same `~/catkin_ws/src` and follow the instructions provided by the repo. I used this [commit](https://github.com/GitSRealpe/iauv_demo/commit/4848c8fe560a5b6e492adde968dbb2573105e5b6). PTAL at `other_repo_modifications`

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
- [stonefish_ros](https://github.com/Michele1996/stonefish_ros) -->

4. Please take a look at the folder `other_repo_modification` to substitute the relative file in the other repo involved in this use case.

5. Build your workspace:
```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

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
# Mock script to cluster the pcl (to be deleted when pcl is fixed)
roslaunch pcl_geometric_primitives_detector mock_labeled_clustering.launch
```

(if debugging) T5
```bash
# to keep separate for debugging - to be included in perception.launch
rosrun pcl_geometric_primitives_detector pcl_geometric_primitives_detector
```
### Mission flow
NB. here action is meant as a single BT

- First part of mission: Survey
1. the BT manager validates the mission file
2. If the mission file is validated, the mission starts
3. the BT manager populates the stack with the entries in mission file
4. BT manager executes stack (FIFO) removing the bt executed from the stack
5. once the stack is empty, the BT manager calls the clustering service commands the auv to go to the luma

- Second part of mission: communication with luma and clustering
6. once it's at the luma, the AUV communicates the clusters
7. from the clusters, the BT manager gets the type of objects detected and where they are

- Third part of mission: population and execution of actions for each object detected
8. the BT manager ask what action it can execute on each object 
9. the BT manager orders these actions based on the priority (in this case, distance AUV-object, from the smaller distance to the biggest)
10. based on priorities, the BT manager populate a new stack with the actions for each object detected
11. the BT manager execute the stack with a FIFO logic and popping the action executed until the stack is empty

## Troubleshooting

### Python version

if checking for `python --version` output 2 (like here)
```bash
$ python --version
Python 2.7.18

```

instead of version 3 (like here)
```bash
$ python --version
Python 3.8.10

```
then, run:
```bash
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 1
sudo update-alternatives --config python
```

### Default
In case there are some problems, please bear in mind that the latest commit tested is in the branch `stable`. So, please:

```bash
cd ~/catkin_ws/src/bt_ace
git checkout stable
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

and try again.
