#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_geometric_primitives_detector/LabeledObjInfo.h>  
#include <utility>


typedef pcl::PointXYZ PointT;

using std::placeholders::_1;

// Define a cylinder model structure
struct CylinderModel {
    Eigen::Vector3f center; // Center of the cylinder
    Eigen::Vector3f axis;   // Axis of the cylinder
    float radius;           // Radius of the cylinder
};

// Function to calculate the distance from a point to a cylinder
double pointToCylinderDistance(const pcl::PointXYZ& point, const CylinderModel& cylinder) {
    // Calculate the vector from the center to the point
    Eigen::Vector3f point_vec(point.x, point.y, point.z);
    Eigen::Vector3f center_vec = cylinder.center;
    Eigen::Vector3f axis_vec = cylinder.axis.normalized();

    // Project the point onto the cylinder axis
    Eigen::Vector3f to_point = point_vec - center_vec;
    float projection_length = to_point.dot(axis_vec);
    Eigen::Vector3f projection = center_vec + projection_length * axis_vec;

    // Calculate the distance to the axis
    float dist_to_axis = (point_vec - projection).norm();
    return std::fabs(dist_to_axis - cylinder.radius);
}

class PrimitiveDetector
{
public:
    PrimitiveDetector() = default;

    // Downsample the cloud to reduce noise
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f); // Adjust leaf size based on your needs
        voxel_grid.filter(*filtered_cloud);
        return filtered_cloud;
    }

    // Compute the distance of a point to the plane defined by the coefficients
    double pointToPlaneDistance(const pcl::PointXYZ& point, const pcl::ModelCoefficients& coefficients)
    {
        // Plane equation: Ax + By + Cz + D = 0
        double A = coefficients.values[0];
        double B = coefficients.values[1];
        double C = coefficients.values[2];
        double D = coefficients.values[3];
        
        return std::fabs(A * point.x + B * point.y + C * point.z + D) / std::sqrt(A * A + B * B + C * C);
    }

    // Compute the distance of a point to the sphere defined by the coefficients
    double pointToSphereDistance(const pcl::PointXYZ& point, const pcl::ModelCoefficients& coefficients)
    {
        // Sphere defined by center (Cx, Cy, Cz) and radius R
        double Cx = coefficients.values[0];
        double Cy = coefficients.values[1];
        double Cz = coefficients.values[2];
        double R = coefficients.values[3];

        double dist = std::sqrt((point.x - Cx) * (point.x - Cx) +
                                 (point.y - Cy) * (point.y - Cy) +
                                 (point.z - Cz) * (point.z - Cz));
        return std::fabs(dist - R);
    }



// Function to generate distinct colors based on plane index
std::tuple<uint8_t, uint8_t, uint8_t> generateColor(int plane_index) {
    float hue = (plane_index * 45) % 360; // Change hue value for each plane (360 degrees / 8 = 45-degree steps)
    float saturation = 1.0, value = 1.0;
    float c = value * saturation;
    float x = c * (1 - fabs(fmod(hue / 60.0, 2) - 1));
    float m = value - c;
    
    float r_prime = 0, g_prime = 0, b_prime = 0;
    if (hue >= 0 && hue < 60) {
        r_prime = c; g_prime = x; b_prime = 0;
    } else if (hue >= 60 && hue < 120) {
        r_prime = x; g_prime = c; b_prime = 0;
    } else if (hue >= 120 && hue < 180) {
        r_prime = 0; g_prime = c; b_prime = x;
    } else if (hue >= 180 && hue < 240) {
        r_prime = 0; g_prime = x; b_prime = c;
    } else if (hue >= 240 && hue < 300) {
        r_prime = x; g_prime = 0; b_prime = c;
    } else if (hue >= 300 && hue < 360) {
        r_prime = c; g_prime = 0; b_prime = x;
    }

    uint8_t r = static_cast<uint8_t>((r_prime + m) * 255);
    uint8_t g = static_cast<uint8_t>((g_prime + m) * 255);
    uint8_t b = static_cast<uint8_t>((b_prime + m) * 255);
    
    return {r, g, b};  // Return the color as an RGB triplet
}


// Function to calculate distance from a point to a cylinder
double pointToCylinderDistance(const pcl::PointXYZ& point, const pcl::ModelCoefficients& coefficients) {
    // The coefficients for the cylinder model are structured as:
    // coefficients.values[0] = x0 (center x)
    // coefficients.values[1] = y0 (center y)
    // coefficients.values[2] = z0 (center z)
    // coefficients.values[3] = a (axis direction x)
    // coefficients.values[4] = b (axis direction y)
    // coefficients.values[5] = c (axis direction z)
    // coefficients.values[6] = r (radius)

    // Center of the cylinder
    Eigen::Vector3f center(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    // Direction of the cylinder
    Eigen::Vector3f axis(coefficients.values[3], coefficients.values[4], coefficients.values[5]);
    // Radius of the cylinder
    float radius = coefficients.values[6];

    // Compute the vector from the center to the point
    Eigen::Vector3f point_vector(point.x, point.y, point.z);
    Eigen::Vector3f to_point = point_vector - center;

    // Project this vector onto the cylinder's axis
    float projection_length = to_point.dot(axis.normalized());
    Eigen::Vector3f projection = projection_length * axis.normalized();

    // Get the nearest point on the cylinder axis
    Eigen::Vector3f nearest_point_on_axis = center + projection;

    // Calculate the distance from the point to the nearest point on the cylinder
    float distance_to_axis = (point_vector - nearest_point_on_axis).norm();

    // The distance to the cylinder surface is the distance to the axis minus the radius
    return std::abs(distance_to_axis - radius);
}

double pointToConeDistance(const pcl::PointXYZ& point, const pcl::ModelCoefficients& coefficients) {
    // Coefficients contain the apex, axis direction, and opening angle
    Eigen::Vector3f apex(coefficients.values[0], coefficients.values[1], coefficients.values[2]); // Apex of the cone
    Eigen::Vector3f axis_dir(coefficients.values[3], coefficients.values[4], coefficients.values[5]); // Axis direction
    float opening_angle = coefficients.values[6]; // Opening angle of the cone

    // Normalize the axis direction
    axis_dir.normalize();

    // Convert the point to an Eigen vector
    Eigen::Vector3f point_vector(point.x, point.y, point.z);

    // Compute the vector from the apex to the point
    Eigen::Vector3f to_point = point_vector - apex;

    // Project the point onto the cone's axis to find the nearest point on the axis
    float projection_length = to_point.dot(axis_dir);
    Eigen::Vector3f closest_point_on_axis = apex + projection_length * axis_dir;

    // Distance from the point to the closest point on the axis
    float distance_to_axis = (point_vector - closest_point_on_axis).norm();

    // Compute the distance from the closest point on the axis to the cone surface
    float distance_to_cone_surface = distance_to_axis - (std::tan(opening_angle) * projection_length);

    // If the projection is behind the apex (inside the cone), we return the distance to the apex
    if (projection_length < 0.0f) {
        return std::abs((point_vector - apex).norm());
    }

    // Return the distance to the cone surface
    return std::abs(distance_to_cone_surface);
}


    // Iteratively fit planes and return the inliers for publishing
std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, double> detectPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud = downsample(cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_planes_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    double total_fitness_score = 0.0; // Total fitness score

    int plane_count = 0;
    while (!remaining_cloud->points.empty())
    {
        // Fit a plane using RANSAC
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01); // Adjust this value if necessary
        seg.setInputCloud(remaining_cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            std::cout << "No more planes detected." << std::endl;
            break;
        }

        std::cout << "Detected Plane: " << plane_count + 1 << " Coefficients: "
                  << coefficients->values[0] << " " << coefficients->values[1] << " "
                  << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

        // Calculate fitness score for this plane
        double fitness_score = 0.0;
        for (int index : inliers->indices)
        {
            fitness_score += pointToPlaneDistance(remaining_cloud->points[index], *coefficients);
        }
        fitness_score /= inliers->indices.size(); // Average distance
        total_fitness_score += fitness_score;

        // Generate a unique color for each plane
        uint8_t r, g, b;
        std::tie(r, g, b) = generateColor(plane_count);  // Generate color for each plane

        // Extract inliers (points forming the detected plane)
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(remaining_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);  // Keep only inliers
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>());
        extract.filter(*plane);

        // Color the points and add them to the final colored planes cloud
        for (const auto& point : plane->points)
        {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = r;
            colored_point.g = g;
            colored_point.b = b;
            colored_planes_cloud->points.push_back(colored_point);
        }

        // Remove inliers from remaining cloud for the next iteration
        extract.setNegative(true);  // Remove inliers from remaining cloud
        extract.filter(*remaining_cloud);

        plane_count++;  // Increment the plane counter
    }

    return {colored_planes_cloud, total_fitness_score}; // Return colored planes and total fitness score
}


    // Detect spheres in the point cloud
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, double> detectSpheres(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr spheres_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud = downsample(cloud);
        double fitness_score = 0.0; // Fitness score for spheres

        // Fit a sphere using RANSAC
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_SPHERE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01); // Adjust this value if necessary
        seg.setInputCloud(remaining_cloud);
        seg.segment(*inliers, *coefficients);

        if (!inliers->indices.empty())
        {
            std::cout << "Detected Sphere Coefficients: "
                      << coefficients->values[0] << " " << coefficients->values[1] << " "
                      << coefficients->values[2] << " Radius: " << coefficients->values[3] << std::endl;

            // Calculate fitness score for this sphere
            for (int index : inliers->indices)
            {
                fitness_score += pointToSphereDistance(remaining_cloud->points[index], *coefficients);
            }
            fitness_score /= inliers->indices.size(); // Average distance

            // Extract inliers (points forming the detected sphere)
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(remaining_cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);  // Keep only inliers
            pcl::PointCloud<pcl::PointXYZ>::Ptr sphere(new pcl::PointCloud<pcl::PointXYZ>());
            extract.filter(*sphere);

            // Add the sphere's points to the final spheres cloud
            *spheres_cloud += *sphere;
        }
        else
        {
            std::cout << "No spheres detected." << std::endl;
        }

        return {spheres_cloud, fitness_score}; // Return spheres and fitness score
    }

// Detect cylinders in the point cloud including normal estimation
std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, double> detectCylinders(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinders_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    double fitness_score = 0.0; // Fitness score for cylinders

    // Estimate point normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Setup normal estimation
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(100); // Set the number of neighbors to use for normal estimation
    ne.compute(*cloud_normals);

    // Fit a cylinder using RANSAC
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER); // Set model type to cylinder
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1); // Weight for normals in distance calculation
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05); // Distance threshold for fitting
    seg.setRadiusLimits(0.0, 1); // Set radius limits for cylinders

    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers, *coefficients);

    if (!inliers->indices.empty()) {
        std::cout << "Detected Cylinder Coefficients: "
                  << coefficients->values[0] << " " << coefficients->values[1] << " "
                  << coefficients->values[2] << " Axis: (" << coefficients->values[3] << ", "
                  << coefficients->values[4] << ", " << coefficients->values[5] << ") Radius: "
                  << coefficients->values[6] << std::endl;

        // Calculate fitness score for this cylinder
        for (int index : inliers->indices) {
            fitness_score += pointToCylinderDistance(cloud->points[index], *coefficients);
        }
        fitness_score /= inliers->indices.size(); // Average distance

        // Extract inliers (points forming the detected cylinder)
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);  // Keep only inliers
        pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder(new pcl::PointCloud<pcl::PointXYZ>());
        extract.filter(*cylinder);

        // Add the cylinder's points to the final cylinders cloud
        *cylinders_cloud += *cylinder;
    } else {
        std::cout << "No cylinders detected." << std::endl;
    }

    return {cylinders_cloud, fitness_score}; // Return cylinders and fitness score
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, double> detectCones(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cones_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    double fitness_score = 0.0; // Fitness score for cones

    // Estimate point normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Setup normal estimation
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(100); // Set the number of neighbors to use for normal estimation
    ne.compute(*cloud_normals);

    // Fit a cone using RANSAC
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CONE); // Set model type to cone
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1); // Weight for normals in distance calculation
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05); // Distance threshold for fitting
    seg.setRadiusLimits(0.0, 1.0); // Set radius limits for cones

    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers, *coefficients);

    if (!inliers->indices.empty()) {
        std::cout << "Detected Cone Coefficients: "
                  << coefficients->values[0] << " " << coefficients->values[1] << " "
                  << coefficients->values[2] << " Apex: (" << coefficients->values[0] << ", "
                  << coefficients->values[1] << ", " << coefficients->values[2] << ") Axis: ("
                  << coefficients->values[3] << ", " << coefficients->values[4] << ", " 
                  << coefficients->values[5] << ") Opening Angle: "
                  << coefficients->values[6] << std::endl;

        // Calculate fitness score for this cone
        for (int index : inliers->indices) {
            fitness_score += pointToConeDistance(cloud->points[index], *coefficients);
        }
        fitness_score /= inliers->indices.size(); // Average distance

        // Extract inliers (points forming the detected cone)
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);  // Keep only inliers
        pcl::PointCloud<pcl::PointXYZ>::Ptr cone(new pcl::PointCloud<pcl::PointXYZ>());
        extract.filter(*cone);

        // Add the cone's points to the final cones cloud
        *cones_cloud += *cone;
    } else {
        std::cout << "No cones detected." << std::endl;
    }

    return {cones_cloud, fitness_score}; // Return cones and fitness score
}


};



class PrimitiveDetectionNode
{
public:
    PrimitiveDetectionNode(ros::NodeHandle& nh)
    {
        // Initialize subscriptions and publishers
        for (int i = 0; i < 10; ++i) {
            std::string topic_name = "/cluster_" + std::to_string(i);
            ros::Subscriber sub = nh.subscribe(topic_name, 10, 
                &PrimitiveDetectionNode::pointCloudCallback, this, i);
            subscriptions_.push_back(sub);
        }

        planes_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("/planes", 10);
        spheres_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("/spheres", 10);
        cylinders_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("/cylinders", 10);
        cones_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("/cones", 10);

        primitive_detector_ = std::make_shared<PrimitiveDetector>();
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, int i)
    {
        ROS_INFO("Processing Cluster: %d", i);

        // Convert PointCloud2 to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // Detect primitives
        auto [planes_cloud, planes_fitness_score, plane_count] = primitive_detector_->detectPlanes(cloud);
        auto [spheres_cloud, spheres_fitness_score] = primitive_detector_->detectSpheres(cloud);
        auto [cylinders_cloud, cylinders_fitness_score] = primitive_detector_->detectCylinders(cloud);
        auto [cones_cloud, cones_fitness_score] = primitive_detector_->detectCones(cloud);

        // Identify the shape with the lowest fitness score
        std::string label;
        pcl::PointCloud<pcl::PointXYZ>::Ptr best_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        double lowest_score = std::numeric_limits<double>::max();

        if (planes_fitness_score < lowest_score) {
            label = "Plane";
            if (plane_count == 6) {
                label = "Box";
            }
            lowest_score = planes_fitness_score;
            pcl::copyPointCloud(*planes_cloud, *best_cloud);
        }
        if (spheres_fitness_score < lowest_score) {
            label = "Sphere";
            lowest_score = spheres_fitness_score;
            pcl::copyPointCloud(*spheres_cloud, *best_cloud);
        }
        if (cylinders_fitness_score < lowest_score) {
            label = "Cylinder";
            lowest_score = cylinders_fitness_score;
            pcl::copyPointCloud(*cylinders_cloud, *best_cloud);
        }
        if (cones_fitness_score < lowest_score) {
            label = "Cone";
            lowest_score = cones_fitness_score;
            pcl::copyPointCloud(*cones_cloud, *best_cloud);
        }

        // Compute centroid and surface point
        geometry_msgs::Point centroid, surface_point;
        if (best_cloud->points.empty()) {
            ROS_ERROR("No points in the selected cluster!");
            return;
        }

        // Calculate centroid
        Eigen::Vector4f centroid_eigen;
        pcl::compute3DCentroid(*best_cloud, centroid_eigen);
        centroid.x = centroid_eigen[0];
        centroid.y = centroid_eigen[1];
        centroid.z = centroid_eigen[2];

        // Choose a representative surface point (first point in the cloud)
        surface_point.x = best_cloud->points[0].x;
        surface_point.y = best_cloud->points[0].y;
        surface_point.z = best_cloud->points[0].z;

        // Populate LabelObjInfo message
        pcl_geometric_primitives_detector::LabeledObjInfo label_msg;
        label_msg.label = label;
        label_msg.centroid = centroid;
        label_msg.surface_point = surface_point;

        // Convert PCL cloud to PointCloud2 for the message
        pcl::toROSMsg(*best_cloud, label_msg.clustered_pointcloud);
        label_msg.clustered_pointcloud.header.frame_id = msg->header.frame_id;

        // Publish the labeled cluster
        std::string topic_name = "/labelled_cluster_" + std::to_string(i);
        ros::Publisher label_publisher = nh_.advertise<pcl_geometric_primitives_detector::LabeledObjInfo>(topic_name, 10);
        label_publisher.publish(label_msg);

        // Log details
        ROS_INFO("Label: %s", label.c_str());
        ROS_INFO("Centroid: (%f, %f, %f)", centroid.x, centroid.y, centroid.z);
        ROS_INFO("Surface Point: (%f, %f, %f)", surface_point.x, surface_point.y, surface_point.z);
    }

private:
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> subscriptions_;
    ros::Publisher planes_publisher_;
    ros::Publisher spheres_publisher_;
    ros::Publisher cylinders_publisher_;
    ros::Publisher cones_publisher_;
    std::shared_ptr<PrimitiveDetector> primitive_detector_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "primitive_detection_node");
    ros::NodeHandle nh;

    PrimitiveDetectionNode node(nh);

    ros::spin();

    return 0;
}
