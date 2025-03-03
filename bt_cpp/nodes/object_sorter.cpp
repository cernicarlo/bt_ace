#include "bt_cpp/object_sorter.h"
#include <cmath>
#include <algorithm>

ObjectSorter::ObjectSorter(const geometry_msgs::Point& reference_point)
    : reference_point_(reference_point) {}

// Helper function to calculate Euclidean distance between two points
double ObjectSorter::calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

void ObjectSorter::sortByDistance(const std::unordered_map<std::string, ObjectInfo>& detected_objects) {
    sorted_objects_.clear();

    // Calculate distances and store them in the vector
    for (const auto& item : detected_objects) {
        const std::string& label = item.first;
        const ObjectInfo& info = item.second;
        double distance = calculateDistance(info.surface_point, reference_point_);
        sorted_objects_.push_back({label, info, distance});
    }

    // Sort the vector by distance in ascending order
    std::sort(sorted_objects_.begin(), sorted_objects_.end(),
              [](const LabeledObject& a, const LabeledObject& b) {
                  return a.distance < b.distance;
              });
}

const std::vector<LabeledObject>& ObjectSorter::getSortedObjects() const {
    return sorted_objects_;
}
