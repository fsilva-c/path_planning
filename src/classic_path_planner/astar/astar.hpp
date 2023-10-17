#include <ros/ros.h>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <queue>
#include <cmath>
#include <algorithm>
#include <array>
#include <geometry_msgs/Point.h>
#include "fs_path_planning/Astar.h"
#include "fs_path_planning/SphereCloud.h"
#include "node.hpp"
#include "discrete_grid.hpp"

class AStar {
private:
    ros::NodeHandle nh_;
    // ros::ServiceServer find_path_service_;
    ros::Subscriber sub_spheres_cloud;
    fs_path_planning::SphereCloud spheres_cloud;
    float threshold;
    DiscreteGrid dg;
    std::vector<int> obstacles;
    
    const std::vector<std::vector<int>> EXPANSION_DIRECTIONS = {
        {-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1},
        {-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 1}, {-1, 1, -1}, {-1, 1, 0}, {-1, 1, 1},
        {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1}, {1, 0, 1}, {1, 1, -1}, {1, 1, 0}, {1, 1, 1}
    };

public:
    AStar(
        ros::NodeHandle &nh_, 
        const float threshold, 
        const DiscreteGrid &dg);

    void init();

    inline void callback_spheres_cloud(const fs_path_planning::SphereCloud &data) {
        spheres_cloud = data; 
    }
    
    float dist_euclidean(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);
    float movement_cost(const Node& current, const Node& neighbor);
    bool is_valid(const Node &node);
    std::vector<Node> get_neighbours(const Node &node);
    std::vector<geometry_msgs::Point> reconstruct_path(const Node &node);
    bool find_path(fs_path_planning::Astar::Request& req, fs_path_planning::Astar::Response& res);
};
