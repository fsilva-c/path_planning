#include "point.hpp"

class DiscreteGrid {
public:
    float resolution;

    DiscreteGrid(const float &resolution) : resolution(resolution) { }

    Point<float> discreteToContinuous(Point<int> &point) {
        float x = point.x * resolution;
        float y = point.y * resolution;
        float z = point.z * resolution;
        return Point<float>{x, y, z};
    }
    Point<int> continuousToDiscrete(Point<float> &point) {
        int x = static_cast<int>(point.x / resolution);
        int y = static_cast<int>(point.y / resolution);
        int z = static_cast<int>(point.z / resolution);
        return Point<int>{x, y, z};
    }
};
