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