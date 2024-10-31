# BT ACE
Repo for automated mission of Girona 1000 with robotic arm to scan and collect data of an area, communicate through luma and interact with objects found.

## What's inside
- `bt_cpp` package
- `pcl_geometric_primitives_detector` package


## Use Case
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
