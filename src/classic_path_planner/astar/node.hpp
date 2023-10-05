#ifndef NODE_H_
#define NODE_H_

#include <geometry_msgs/Point.h>

struct Node {
    geometry_msgs::Point position;
    Node* parent;
    float g;
    float h;
    float f;
    Node() : parent(nullptr), g(0.0f), h(0.0f), f(0.0f) { }
    Node(const geometry_msgs::Point& position) : 
        position(position), parent(nullptr), g(0.0f), h(0.0f), f(0.0f) { }
    bool operator==(const Node& other) const {
        return position == other.position;
    }
    bool operator<(const Node& other) const {
        return f > other.f;
    }
};

#endif