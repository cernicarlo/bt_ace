#ifndef OBJECT_SORTER_H
#define OBJECT_SORTER_H

#include <geometry_msgs/Point.h>
#include <unordered_map>
#include <string>
#include <vector>
#include "bt_cpp/primitive_detection_client.h"

// Struct to represent an object with a label and its distance
struct LabeledObject {
    std::string label;
    ObjectInfo info;
    double distance;
};

class ObjectSorter {
public:
    ObjectSorter(const geometry_msgs::Point& reference_point);
    
    // Method to sort detected objects by distance from the reference point
    void sortByDistance(const std::unordered_map<std::string, ObjectInfo>& detected_objects);

    // Method to retrieve the sorted objects
    const std::vector<LabeledObject>& getSortedObjects() const;

private:
    // Reference point (e.g., luma_station)
    geometry_msgs::Point reference_point_;
    
    // Vector to store sorted objects with distances
    std::vector<LabeledObject> sorted_objects_;

    // Helper function to calculate distance
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const;
};

#endif // OBJECT_SORTER_H
