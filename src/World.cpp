//
// Created by vishnu on 9/28/19.
//

#include "World.hpp"



DStarLite::World::World(int rows, int columns) : _rows(rows), _columns(columns), start(-1, -1), goal(-1, -1) {
    grid.resize(rows, std::vector<int>(columns, 0));
}

std::ostream &operator<<(std::ostream &os, const DStarLite::World &world) {
    os << "_rows: " << world._rows << " _columns: " << world._columns << " start: " << world.start << " goal: "
       << world.goal;
    return os;
}

void DStarLite::World::addObstacle(const Node &node1, const Node &node2) {
auto x1 = std::min(node1._x, node2._x);
  auto x2 = std::max(node1._x, node2._x);
  auto y1 = std::min(node1._y, node2._y);
  auto y2 = std::max(node1._y, node2._y);
  for (auto row = y1; row <= y2; ++row) {
    for (auto col = x1; col <= x2; ++col) {
      grid[row][col] = 1;
    }
  }
}

std::vector<DStarLite::Node> DStarLite::World::getNeighbours(const Node &node) {
    std::vector<Node> neighbors;
    for(const auto& i : {-1,0,1}){
        for(const auto& j:{-1,0,1}){
            if(i==j) continue;
            neighbors.emplace_back(node._x+i, node._y+j);
        }
    }
    return neighbors;
}

bool DStarLite::World::isValid(const Node &node) {
    return (node._x < _columns) && (node._y < _rows) && (node._y >= 0) && (grid[node._y][node._x]);
}

DStarLite::World::World(const World &other) {
    _rows = other._rows;
    _columns = other._columns;
    start = other.start;
    goal = other.goal;
    grid = other.grid;
}


DStarLite::World::~World() = default;
