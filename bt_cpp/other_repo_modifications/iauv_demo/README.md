# Modifications
* `iauv_description/launch/core/girona1000_base.launch.core` 
```xml
<!-- <node name="captain" pkg="cola2_control" type="captain_node" output="screen">
        <param name="vehicle_config_launch_mission_package" value="cola2_$(arg robot_name)"/>
    </node> -->
```

* `iauv_description/launch/single_robot.launch` 
```xml
<arg name="enable_joystick" default="false"/>
```

* `iauv_motion_planner/scripts/request_path.py` (to go to LUMA)
```bash
+        req.planner = GetPathRequest.SIMPLE
+
+        # LUMA
+        req.start.position.x = -7.5
+        req.start.position.y = 3
+        req.start.position.z = 2.25
+
+        req.start.orientation.w = 0.707
+        req.start.orientation.z = -0.707
+
+        req.goal.position.x = -10.0
+        req.goal.position.y = 3
+        req.goal.position.z = 6.25
         # req.goal.orientation.w = 0.707
         # req.goal.orientation.z = 0.707
         # req.goal.orientation.w = 0.924
         # req.goal.orientation.z = 0.383
-        req.goal.orientation.w = 1
-        req.goal.orientation.z = 0
+        req.goal.orientation.w = 0.707
+        req.goal.orientation.z = -0.707
```