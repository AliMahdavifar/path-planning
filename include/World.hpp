//
// Created by vishnu on 9/28/19.
//

#pragma once


#include <vector>
#include <ostream>

#include "Node.hpp"

namespace DStarLite {
struct World {
    int _rows{};
    int _columns{};


    Node start;
    Node goal;

    std::vector<std::vector<int>> grid;
    World():_rows(50),_columns(50) {}
    World(int rows, int columns);
    World(const World& other);
    void addObstacle(const Node &node1, const Node &node2);

    bool isValid(const Node &node);
    static std::vector<Node> getNeighbours(const Node& node);
    friend std::ostream &operator<<(std::ostream &os, const World &world);

    virtual ~World();

};


}