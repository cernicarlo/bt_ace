# Modifications
* `/scripts/pursuit_controller.cpp` 
```bash
@@ -68,6 +68,7 @@ public:
     Eigen::Array3d p0, p1, sph;
     double radius;
     int waypoint_index;
+    size_t max_waypoints_;
 
     PursuitController(std::string name) : as_(nh_, name, false), action_name_(name)
     {
@@ -177,6 +178,8 @@ public:
         auto goal = as_.acceptNewGoal();
         path = goal->path;
         radius = goal->radius;
+        max_waypoints_ = path.poses.size();
+        ROS_INFO("max_waypoints: %ld", max_waypoints_);
 
         sphere_m.scale.x = radius * 2;
         sphere_m.scale.y = radius * 2;
@@ -330,7 +333,7 @@ public:
             {
                 // if distance is less than 0.01 then get next point in path
                 waypoint_index++;
-                std ::cout << "index" << waypoint_index << "\n";
+                std ::cout << "index " << waypoint_index << "\n";
             }
             break;

@@ -338,17 +341,6 @@ public:
             break;
         }
 
-        if (waypoint_index < path.poses.size())
-        {
-            std::cout << "last waypoint reached" << "\n";
-            markers.markers.at(1).pose = setPoint;
-            markers.markers.at(2).pose = path.poses[waypoint_index].pose;
-            setPoint.orientation = path.poses[waypoint_index].pose.orientation;
-        }
-        else
-        {
-            setPoint = path.poses.back().pose;
-        }
 
         pubviz.publish(markers);

@@ -395,6 +387,24 @@ public:
         sendVelCOLA2(pid_err, err_yaw);
 
         prev_t = ros::Time::now();
+
+        if (waypoint_index < max_waypoints_)
+        {
+            ROS_INFO("reached waypoint: %d/%ld", waypoint_index, max_waypoints_);
+            markers.markers.at(1).pose = setPoint;
+            markers.markers.at(2).pose = path.poses[waypoint_index].pose;
+            setPoint.orientation = path.poses[waypoint_index].pose.orientation;
+        }
+        else
+        {
+            setPoint = path.poses.back().pose;
+            ROS_INFO("last waypoint reached: %d/%ld", waypoint_index, max_waypoints_);
+            setPoint = path.poses.back().pose;
+            result_.success = true;
+            as_.setSucceeded(result_);  // Publish success
+            timer_.stop();  // Stop the timer
+            return;
+        }
     }
 };



```
