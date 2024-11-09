# PCL geometric primitives detector
ros run to run this package
```bash
ros run pcl_geometric_primitives_detector pcl_geometric_primitives_detector
```

This code implements a PrimitiveDetector class for detecting geometric primitives (planes, spheres, cylinders, and cones) in a 3D point cloud, primarily using RANSAC-based segmentation from the Point Cloud Library (PCL). The detected shapes are published as [LabeledObjInfo](https://github.com/cernicarlo/bt_ace/blob/master/pcl_geometric_primitives_detector/msg/LabeledObjInfo.msg) message.
## Perception and Knowledge Representation Scripts
The launch file `perception.launch` in the launch folder runs the following scripts:
```bash
affordanceGraphN.py
marker_llm.py
object_sensing_ACE.py
pcd_compressor.py
```
The launch file also creates the following parameters containing the mission, graph and taxonomy files
```bash
mission_file
affordance_graph
taxonomy
```
Don't forget to add "/" in the scripts or cpp code to access them like:
```python
mission_file = rospy.get_param('/mission_file')
```
### Knowledge Graph
The affordanceGraphN script creates the graph based on the affordance_graph.yaml file. 
It has:
*  a service called "validate_mission", that takes in input the mission file in the ros_param and checks if the mission is possible (it returns True or False)
*  a service called "display_graph" which displays the actual graph
*  a service called "display_gui" which allows a user to add an object and new affordances (object, action) in the graph
*  a service called "query" checks the relation between nodes in the graph as triplet (subject, target, action) ex. (StereoPair, Survey, allows)
*  a Service called "get_actions" for retrieving possible actions for an object. Takes in input the label of the object

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
