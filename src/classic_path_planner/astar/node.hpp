#include <geometry_msgs/Point.h>

struct Node {
    geometry_msgs::Point position;
    float g; // custo do caminho do nó inicial até este nó...
    float h; // heurística...
    float f; // f = g + h...
    Node* parent;

    Node() : parent(nullptr), g(0.0f), h(0.0f), f(0.0f) { }
    Node(const geometry_msgs::Point& position) : 
        position(position), parent(nullptr), g(0.0f), h(0.0f), f(0.0f) { }
        
    bool operator==(const Node& other) const;
    bool operator!=(const Node& other) const;
    bool operator<(const Node& other) const;
    bool operator<=(const Node& other) const;
};

struct CostComparator
{
  bool operator()(const Node &n1, const Node &n2) const;
};

struct HashFunction
{
  bool operator()(const Node &n) const;
};
