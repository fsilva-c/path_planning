#include <ros/ros.h>
#include <vector>
#include "fs_path_planning/Astar.h"
#include "node.hpp"
#include "discrete_grid.hpp"

class AStar {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer find_path_service_;
    ros::Subscriber sub_obstacles;
    float threshold;
    DiscreteGrid dg;
    std::vector<int> obstacles;

public:
    AStar(
        ros::NodeHandle &nh_, 
        const float threshold, 
        const DiscreteGrid &dg);

    void init();
    bool test();
    
    float heuristic(const Node &node, const Node &goal);
    bool is_valid(const Node &node);
    std::vector<Node> get_neighbours(const Node &node);
    bool find_path(fs_path_planning::Astar::Request& req, fs_path_planning::Astar::Response& res);
};