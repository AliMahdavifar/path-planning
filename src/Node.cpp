//
// Created by vishnu on 9/28/19.
//

#include "Node.hpp"

DStarLite::Node::Node(int x, int y) : _x(x), _y(y) {}

DStarLite::Node::Node(const Node &n) {
    _x = n._x;
    _y = n._y;
    _key = n._key;
}

void DStarLite::Node::setKey(std::pair<double, double> &k) {
    _key = k;
}

std::pair<double, double> DStarLite::Node::getKey() {
    return _key;
}