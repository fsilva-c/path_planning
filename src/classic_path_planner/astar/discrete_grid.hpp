#include <geometry_msgs/Point.h>

class DiscreteGrid {
public:
    float resolution;

    DiscreteGrid(const float &resolution) : resolution(resolution) { }

    geometry_msgs::Point discrete_to_continuous(const geometry_msgs::Point &point) {
        geometry_msgs::Point p;
        p.x = point.x * resolution;
        p.y = point.y * resolution;
        p.z = point.z * resolution;
        return p;
    }
    geometry_msgs::Point continuous_to_discrete(const geometry_msgs::Point &point) {
        geometry_msgs::Point p;
        p.x = std::floor(point.x / resolution);
        p.y = std::floor(point.y / resolution);
        p.z = std::floor(point.z / resolution);
        return p;
    }
};
