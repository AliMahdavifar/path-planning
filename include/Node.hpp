//
// Created by vishnu on 9/28/19.
//

#pragma once


#include <utility>
#include <ostream>

namespace DStarLite {
    class Node {
    public:
        int _x{}, _y{};
        std::pair<double, double> _key = std::make_pair(0.f, 0.f);

        Node() = default;

        Node(int x, int y);

        Node(const Node &n);

        void setKey(std::pair<double, double> &k);

        std::pair<double, double> getKey();

        friend Node operator+(const Node &l, const Node &r);

        friend bool operator==(const Node &l, const Node &r);

        friend bool operator!=(const Node &l, const Node &r);

        friend bool operator<(const Node &l, const std::pair<double, double> r) {
            if (l._key.first < r.first) {
                return true;
            } else {
                return (l._key.first == r.first) && (l._key.second < r.second);
            }
        }

        struct compare {
            bool operator()(const Node &l, const Node &r) {
                if (l._key.first < r._key.first) {
                    return false;
                } else {
                    if ((l._key.first == r._key.first) && (l._key.second < r._key.second))
                        return false;
                    else
                        return true;
                }
            }
        };

        friend std::ostream &operator<<(std::ostream &os, const Node &node);

        ~Node();
    };

}



