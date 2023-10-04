#ifndef NODE_H_
#define NODE_H_

#include "point.hpp"

struct Node {
    Point<float> position;
    float g;
    float h;
    float f;
    Node* parent;
    bool operator<(const Node& other) const {
        return f < other.f;
    }
};

#endif