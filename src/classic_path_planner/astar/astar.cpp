#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
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
    this->find_path_service_ = nh_.advertiseService("path_finder", &AStar::find_path, this);
}

bool AStar::is_valid(const Node &node) {
    if (node.position.z < 0.5 or node.position.z > 6.0) {
        return false;
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
    const std::array<int, 3> deltas[] = {
        {-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, 
        {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};
    std::vector<Node> neighbors;
    for (const std::array<int, 3>& delta : deltas) {
        int dx = delta[0];
        int dy = delta[1];
        int dz = delta[2];

        geometry_msgs::Point neighbor;
        neighbor.x = node.position.x + dx;
        neighbor.y = node.position.y + dy;
        neighbor.z = node.position.z + dz;

        if (is_valid(neighbor)) {
            neighbors.push_back(Node(neighbor));
        }
    }

    return neighbors;
}

std::vector<geometry_msgs::Point> AStar::reconstruct_path(const Node &node) {
    std::vector<geometry_msgs::Point> path;
    const Node* current = &node;
    while (current != nullptr) {
        path.push_back(dg.discrete_to_continuous(current->position));
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

bool AStar::find_path(fs_path_planning::Astar::Request& req, fs_path_planning::Astar::Response& res) {
    Node start_node = Node(dg.continuous_to_discrete(req.start));
    Node goal_node = Node(dg.continuous_to_discrete(req.goal));

    std::priority_queue<Node> open_set;
    open_set.push(start_node);

    std::cout << "calculando caminho astar..." << '\n';

    while (!open_set.empty()) {
        Node current = open_set.top();

        if (current.position == goal_node.position) {
            std::cout << "ACHOU O CAMINHO!..." << '\n';
            res.path.points = reconstruct_path(current);
            return true;
        }
        
        open_set.pop();

        std::vector<Node> neighbors = get_neighbours(current); // trocar para current->position

        for (Node& neighbor : neighbors) {
            float tentative_g = current.g + heuristic(current, neighbor);

            if (tentative_g < neighbor.g) {
                neighbor.parent = &current;
                neighbor.g = tentative_g;
                neighbor.h = heuristic(neighbor, goal_node);
                neighbor.f = neighbor.g + neighbor.h;

                open_set.push(neighbor);
            }
        }
    }

    std::cout << "retornando..." << '\n';

    return true; // temp...
    // return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar_node");
    ros::NodeHandle nh;
    DiscreteGrid dg(0.5);
    AStar astar(nh, 0.5, dg);
    ROS_INFO("Serviço 'path_finder' pronto para ser chamado.");
    ros::spin();
    return 0;
}
