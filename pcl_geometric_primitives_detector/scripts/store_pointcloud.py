import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class PointCloudSaver:
    def __init__(self):
        self.pointcloud_name = "cluster_0"
        rospy.init_node('pointcloud_saver', anonymous=True)
        rospy.Subscriber(f'/{self.pointcloud_name}', PointCloud2, self.callback)
        rospy.loginfo("Waiting for point cloud data...")

    def callback(self, msg):
        # Convert ROS PointCloud2 message to numpy array
        pc_data = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        
        # Create an Open3D point cloud and save it
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(pc_data)
        o3d.io.write_point_cloud(f"{self.pointcloud_name}.pcd", o3d_cloud)
        rospy.loginfo(f"Point cloud saved as {self.pointcloud_name}.pcd")

if __name__ == '__main__':
    try:
        PointCloudSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
