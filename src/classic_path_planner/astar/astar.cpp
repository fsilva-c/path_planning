#include "astar.hpp"

/* ##### ROS ##### */
void update_callbacks() {
    ros::spinOnce();
    ros::Rate(10.0).sleep();
}

/* ##### NODE ##### */
bool Node::operator==(const Node& other) const {
    return position == other.position;
}

bool Node::operator!=(const Node& other) const {
    return position != other.position;
}

bool Node::operator<(const Node& other) const {
    if (f == other.f) {
        return h < other.h;
    }

    return f < other.f;
}

bool Node::operator<=(const Node& other) const {
    if (f == other.f) {
        return h <= other.h;
    }
    
    return f <= other.f;
}

bool CostComparator::operator()(const Node &n1, const Node &n2) const {
  if (n1.f == n2.f) {
    return n1.h > n2.h;
  }

  return n1.f > n2.f;
}

bool HashFunction::operator()(const Node &n) const {
  using std::hash;
  return ((hash<int>()(n.position.x) ^ (hash<int>()(n.position.y) << 1)) >> 1) ^ (hash<int>()(n.position.z) << 1);
}

/* ##### ASTAR ##### */
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
    // find_path_service_ = nh_.advertiseService("path_finder", &AStar::find_path, this);
    sub_spheres_cloud = nh_.subscribe(
        "/fspp_classical/spheres_cloud", 100, &AStar::callback_spheres_cloud, this);
}

float AStar::dist_euclidean(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

float AStar::movement_cost(const Node& current, const Node& neighbor) {
    float dx = std::abs(current.position.x - neighbor.position.x);
    float dy = std::abs(current.position.y - neighbor.position.y);
    float dz = std::abs(current.position.z - neighbor.position.z);
    float cost = dx + dy + 3.0 * dz; // peso no eixo z...
    return cost;
}


bool AStar::is_valid(const Node &node) {
    auto continuous_point = dg.discrete_to_continuous(node.position);
    if (continuous_point.z < 0.5) {
        return false;
    }

    for (const auto &sphere : spheres_cloud.spheres) {
        auto distance = dist_euclidean(sphere.center, continuous_point);
        if (distance < sphere.radius * 3.0) {
            // std::cout << "node inválido p distancia... " << distance << ' ' << sphere.radius << '\n';
            return false;
        }
    }
    
    return true;
}

std::vector<Node> AStar::get_neighbours(const Node &node) {
    std::vector<Node> neighbors;
    for (const auto &delta : EXPANSION_DIRECTIONS) {
        Node neighbor;
        neighbor.position.x = node.position.x + delta[0];
        neighbor.position.y = node.position.y + delta[1];
        neighbor.position.z = node.position.z + delta[2];

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
    auto time_start = ros::Time::now();
    
    auto start_discrete = dg.continuous_to_discrete(req.start);
    auto goal_discrete = dg.continuous_to_discrete(req.goal);

    Node goal_node;
    goal_node.position = goal_discrete;
    if (!is_valid(goal_node)) {
        ROS_ERROR("Ponto de destino inválido.");
        return false;
    }

    std::priority_queue<Node, std::vector<Node>, CostComparator> frontier;
    std::unordered_set<Node, HashFunction> open_set;
    std::unordered_set<Node, HashFunction>  closed_set;

    Node start;
    start.position = start_discrete;
    start.g = 0;
    start.h = dist_euclidean(start_discrete, goal_discrete);
    start.f = start.g + start.h;
    frontier.push(start);
    open_set.insert(start);

    while (!frontier.empty()) {
        Node current = frontier.top();
        frontier.pop();

        auto time_now = ros::Time::now();
        if (time_now.toSec() - time_start.toSec() > 5) {
            ROS_INFO("Timeout! Nenhum caminho encontrado...");
            return false;
        }

        if (current.position == goal_discrete) { // path found...
            res.path.points = reconstruct_path(current);
            return true;
        }
        
        open_set.erase(current);
        closed_set.insert(current);

        auto neighbours = get_neighbours(current);
        if (!neighbours.size()) {
            ROS_INFO("PONTO SEM VIZINHOS VÁLIDOS!!");
            return false;
        }
        for (auto &neighbor : neighbours) {
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;  // já foi explorado...
            }

            float tentative_g = current.g + movement_cost(current, neighbor);

            if (open_set.find(neighbor) == open_set.end() || tentative_g < neighbor.g) {
                neighbor.g = tentative_g;
                neighbor.h = dist_euclidean(neighbor.position, goal_discrete);
                neighbor.parent = new Node(current);

                if (open_set.find(neighbor) == open_set.end()) {
                    frontier.push(neighbor);
                    open_set.insert(neighbor);
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
    DiscreteGrid dg(0.5);
    AStar astar(nh, 0.5, dg);
    ros::ServiceServer service = nh.advertiseService("path_finder", &AStar::find_path, &astar);
    ROS_INFO("Serviço 'path_finder' pronto para ser chamado.");


    /* test... */
    /*
    std::vector<geometry_msgs::Point> valid_points;
    geometry_msgs::Point center;
    center.x = 0.0;
    center.y = 0.0;
    center.z = 1.22;
    const float radius = 14.0;
    for (double x = center.x - radius; x <= center.x + radius; x += 1.0) {
        for (double y = center.y - radius; y <= center.y + radius; y += 1.0) {
            for (double z = center.z - radius; z <= center.z + radius; z += 1.0) {
                geometry_msgs::Point point;
                point.x = x;
                point.y = y;
                point.z = z;

                // Verifique se o ponto é válido usando a função is_valid
                if (astar.is_valid(point)) {
                    std::cout << point << '\n';
                    // valid_points.push_back(point);
                }
            }
        }
    }
    */

    ros::spin();
    return 0;
}
