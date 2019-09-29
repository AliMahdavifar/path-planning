//
// Created by vishnu on 9/28/19.
//

#pragma once

#include <utility>

#include "PriorityQueue.hpp"
#include "Node.hpp"
#include "World.hpp"

namespace DStarLite {
    class DStarLite {
    public:
        explicit DStarLite(const World &world, std::vector<std::vector<double>> g,
                  std::vector<std::vector<double>> rhs, std::vector<std::vector<double>> cost, int km);

        std::pair<double, double> calculateKey(Node node);

        void initialize();

        void updateVertex(Node u);

        void computeShortestPath();



    private:
        World world;
        std::vector<std::vector<double>> g;
        std::vector<std::vector<double>> rhs;
        std::vector<std::vector<double>> cost;
        int km;
        PriorityQueue<Node> U;
        double heuristic(const Node& n);


    };
}



