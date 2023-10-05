#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Point.h>
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
    
    float heuristic(const Node &node, const Node &goal);
    bool is_valid(const Node &node);
    std::vector<Node> get_neighbours(const Node &node);
    std::vector<geometry_msgs::Point> reconstruct_path(const Node &node);
    bool find_path(fs_path_planning::Astar::Request& req, fs_path_planning::Astar::Response& res);
};