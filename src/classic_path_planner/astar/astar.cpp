#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <array>
#include "astar.hpp"

AStar::AStar(
    ros::NodeHandle &nh_, 
    const float threshold, 
    const DiscreteGrid &dg) : 
    nh_(nh_),
    threshold(threshold),
    dg(dg) {
    this->init();
}

void AStar::init() { 
    find_path_service_ = nh_.advertiseService("path_finder", &AStar::find_path, this);
    sub_spheres_cloud = nh_.subscribe(
        "/fspp_classical/spheres_cloud", 100, &AStar::callback_spheres_cloud, this);
}

bool AStar::is_valid(const Node &node) {
    auto continuous_point = dg.discrete_to_continuous(node.position);
    if (continuous_point.z < 0.5 || continuous_point.z > 6.0) {
        return false;
    }

    for (const auto &sphere : spheres_cloud.spheres) {
        auto distance = std::sqrt(
            std::pow(sphere.center.x - continuous_point.x, 2) +
            std::pow(sphere.center.y - continuous_point.y, 2) +
            std::pow(sphere.center.z - continuous_point.z, 2));
        if (distance < sphere.radius + threshold * 2) {
            return false;
        }
    }
    return true;
}

float AStar::heuristic(const Node &node, const Node &goal) {
    return std::sqrt(
        std::pow(node.position.x - goal.position.x, 2) +
        std::pow(node.position.y - goal.position.y, 2) +
        std::pow(node.position.z - goal.position.z, 2));
}

std::vector<Node> AStar::get_neighbours(const Node &node) {
    /*
    const std::array<int, 3> deltas[] = {
        {-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1},
        {-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 1}, {-1, 1, -1}, {-1, 1, 0}, {-1, 1, 1},
        {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1}, {1, 0, 1}, {1, 1, -1}, {1, 1, 0}, {1, 1, 1}};
    */
    const std::array<int, 3> deltas[] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
        {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
    };
    std::vector<Node> neighbors;

    for (const std::array<int, 3>& delta : deltas) {
        geometry_msgs::Point p;
        p.x = node.position.x + delta[0];
        p.y = node.position.y + delta[1];
        p.z = node.position.z + delta[2];
        auto neighbor = Node(p);
        if (is_valid(neighbor)) {
            neighbors.push_back(neighbor);
        }
    }

    return neighbors;
}

std::vector<geometry_msgs::Point> AStar::reconstruct_path(const Node &node) {
    std::vector<geometry_msgs::Point> path;
    auto current = &node;
    while (current) {
        path.push_back(dg.discrete_to_continuous(current->position));
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

bool AStar::find_path(fs_path_planning::Astar::Request& req, fs_path_planning::Astar::Response& res) {
    Node start_node = Node(dg.continuous_to_discrete(req.start));
    Node goal_node = Node(dg.continuous_to_discrete(req.goal));

    std::priority_queue<Node> frontier;
    std::vector<geometry_msgs::Point> frontier_vec;

    frontier.push(start_node);
    frontier_vec.push_back(start_node.position);

    while (!frontier.empty()) {
        Node current_node = frontier.top();
        frontier.pop();

        if (current_node.position == goal_node.position) {
            res.path.points = reconstruct_path(current_node);
            return true;
        }

        for (Node neighbour : get_neighbours(current_node)) {
            float new_cost = current_node.g + heuristic(current_node, neighbour);
            bool in_frontier = (std::find(frontier_vec.begin(), frontier_vec.end(), neighbour.position) != frontier_vec.end());

            if (!in_frontier || new_cost < neighbour.g) {
                neighbour.g = new_cost;
                neighbour.h = heuristic(neighbour, goal_node);
                neighbour.f = new_cost + neighbour.h;
                neighbour.parent = new Node(current_node);

                if (!in_frontier) {
                    frontier.push(neighbour);
                    frontier_vec.push_back(neighbour.position);
                }
            }
        }
    }

    ROS_INFO("Nenhum caminho encontrado...");
    return false; // path not found...
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar_node");
    ros::NodeHandle nh;

    ros::spinOnce();
    DiscreteGrid dg(0.5);
    AStar astar(nh, 0.5, dg);
    ROS_INFO("Serviço 'path_finder' pronto para ser chamado.");
    ros::spin();
    return 0;
}
