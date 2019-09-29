//
// Created by vishnu on 9/28/19.
//

#include "DStarLite.hpp"

#include <utility>
#include <cmath>


DStarLite::Node DStarLite::operator+(const Node &l, const Node &r) {
    return Node(l._x + r._x, l._y + r._y);
}

bool DStarLite::operator==(const Node &l, const Node &r) {
    return ((l._x == r._x) && (l._y == r._y));
}

bool DStarLite::operator!=(const Node &l, const Node &r) {
    return l != r;
}

std::ostream &DStarLite::operator<<(std::ostream &os, const Node &node) {
    os << "_x: " << node._x << " _y: " << node._y;
    return os;
}

DStarLite::Node::~Node() = default;

DStarLite::DStarLite::DStarLite(const World &world, std::vector<std::vector<double>> g,
                                std::vector<std::vector<double>> rhs,
                                std::vector<std::vector<double>> cost, int km)
        : world(world), g(std::move(g)), rhs(std::move(rhs)), cost(std::move(cost)), km(km) {}



double DStarLite::DStarLite::heuristic(const Node &n) {
    return (sqrt(pow(n._x - world.start._x, 2) + pow(n._y - world.start._y, 2)));
}

std::pair<double, double> DStarLite::DStarLite::calculateKey(Node node) {
    double minVal = g[node._x][node._y] > rhs[node._x][node._y] ? rhs[node._x][node._y] : g[node._x][node._y];
    double key1 = minVal + heuristic(node)+km;
    double key2 = minVal;
    return {key1,key2};
}

void DStarLite::DStarLite::initialize() {

    rhs[world.goal._x][world.goal._y] = 0;
    auto k = calculateKey(world.goal);
    world.goal.setKey(k);

    U.push(world.goal);

}

void DStarLite::DStarLite::updateVertex(Node u) {
    if(u!=world.goal){
        auto neighbors = world.getNeighbours(u);
        std::vector<double> values;
        for(auto n : neighbors) values.push_back(g[n._x][n._y]+cost[n._x][n._y]);
        rhs[u._x][u._y] = *std::min_element(std::begin(values), std::end(values));

        if( u == U.find(u)) U.remove(u);
        if(g[u._x][u._y]!=rhs[u._x][u._y]){
            auto k = calculateKey(u);
            u.setKey(k);
            U.push(u);
        }
    }
}

void DStarLite::DStarLite::computeShortestPath() {

    while(U.top() < calculateKey(world.start) || rhs[world.start._x][world.start._y]!=g[world.start._x][world.start._y]){
        Node u = U.get();
        if(u < calculateKey(u)){
            auto k_new = calculateKey(u);
            u.setKey(k_new);
            U.push(u);
        }

        else if(g[u._x][u._y] > rhs[u._x][u._y]){
            g[u._x][u._y] = rhs[u._x][u._y];
            auto neighbors = world.getNeighbours(u); // this step should only take predecessors, but the world is un-directed graph
            for(auto n : neighbors) if (cost[n._x][n._y] == 1) updateVertex(n);
        }
    }
}
