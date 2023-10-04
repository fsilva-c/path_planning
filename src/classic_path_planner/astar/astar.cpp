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

bool AStar::is_valid(const Node &node)
{
    if (node.position.z < 0.5 or node.position.z > 6.0) {
        return false;
    }

    return true;
}

bool AStar::find_path(fs_path_planning::Astar::Request& req, fs_path_planning::Astar::Response& res) {
    ROS_INFO("REQ!");
    return true;
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

/*
float calculateDistance(const Point& p1, const Point& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool isValidNode(const Point& point) {
    if (point.z >= 0.5 && point.z <= 6.0) {
        return false;
    }

    auto continuosPoint = discreteToContinuous(point);
    auto obstacles = std::vector<Sphere>();
    for (const Sphere& sphere : obstacles) {
        float distance = std::sqrt(
            std::pow(sphere.center.x - continuosPoint.x, 2) +
            std::pow(sphere.center.y - continuosPoint.y, 2) +
            std::pow(sphere.center.z - continuosPoint.z, 2)
        );

        if (distance < sphere.radius + 0.5 * 2) {
            return false;
        }
    }

    return true;
}

// Obtém os vizinhos válidos de um nó
std::vector<Point> getNeighbors(const Point& point, const std::vector<Sphere>& spheres) {
    std::vector<Point> neighbors;

    for (float dx = -0.5; dx <= 0.5; dx += 0.5) {
        for (float dy = -0.5; dy <= 0.5; dy += 0.5) {
            for (float dz = -0.5; dz <= 0.5; dz += 0.5) {
                Point neighborPoint = {point.x + dx, point.y + dy, point.z + dz};

                if (isValidNode(neighborPoint)) {
                    // Verifique a distância para as esferas
                    bool valid = true;
                    for (const Sphere& sphere : spheres) {
                        if (calculateDistance(neighborPoint, sphere.center) < sphere.radius * 2.0) {
                            valid = false;
                            break;
                        }
                    }

                    if (valid) {
                        neighbors.push_back(neighborPoint);
                    }
                }
            }
        }
    }

    return neighbors;
}

// Função de comparação para a fila de prioridade
struct NodeCompare {
    bool operator()(const Node* lhs, const Node* rhs) const {
        return lhs->f > rhs->f;
    }
};

// Função A* para encontrar o caminho
std::vector<Point> astar(const Point& start, const Point& goal, const std::vector<Sphere>& spheres) {
    std::priority_queue<Node*, std::vector<Node*>, NodeCompare> openSet;
    Node* startNode = new Node{start, 0.0, calculateDistance(start, goal), calculateDistance(start, goal), nullptr};
    openSet.push(startNode);

    std::vector<Node*> closedSet;

    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();
        closedSet.push_back(current);

        if (calculateDistance(current->point, goal) < 0.5) {
            // Chegou ao objetivo
            // Reconstrua o caminho a partir do objetivo
            std::vector<Point> path;
            while (current != nullptr) {
                path.push_back(current->point);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());

            // Libere a memória dos nós alocados dinamicamente
            for (Node* node : closedSet) {
                delete node;
            }
            return path;
        }

        std::vector<Point> neighbors = getNeighbors(current->point, spheres);

        // Gerar vizinhos
        for (const Point& neighborPoint : neighbors) {
            Node* neighbor = new Node{neighborPoint, current->g + 1.0, calculateDistance(neighborPoint, goal),
                                      current->g + 1.0 + calculateDistance(neighborPoint, goal), current};
            bool skipNeighbor = false;

            // Verifique se o vizinho já está na lista closedSet
            for (Node* closedNode : closedSet) {
                if (neighbor->point.x == closedNode->point.x && neighbor->point.y == closedNode->point.y &&
                    neighbor->point.z == closedNode->point.z) {
                    skipNeighbor = true;
                    delete neighbor;
                    break;
                }
            }

            if (!skipNeighbor) {
                openSet.push(neighbor);
            }
        }
    }

    // Caminho não encontrado
    // Libere a memória dos nós alocados dinamicamente
    for (Node* node : closedSet) {
        delete node;
    }
    return std::vector<Point>();
}

int main() {
    // Defina o ponto de início e o objetivo como pontos 3D
    Point start = {0.0, 0.0, 0.5};
    Point goal = {5.0, 5.0, 6.0};

    // Defina as esferas
    std::vector<Sphere> spheres;
    // Preencha a lista de esferas com as informações relevantes

    // Encontre o caminho
    std::vector<Point> path = astar(start, goal, spheres);

    if (!path.empty()) {
        std::cout << "Caminho encontrado:" << std::endl;
        for (const Point& point : path) {
            std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ") ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "Caminho não encontrado." << std::endl;
    }

    return 0;
}
*/
