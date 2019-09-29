//
// Created by vishnu on 9/27/19.
//

#pragma once

#include <iostream>
#include <queue>
#include <algorithm>


template<typename PQElement, class Container = std::vector<PQElement>, class Compare = std::greater<typename Container::value_type>>
struct PriorityQueue : public std::priority_queue<PQElement, Container, Compare> {
//    /**
//     * @brief Add and element to the queue
//     *
//     * @param item
//     */
//    inline void push(PQElement item) {
//        this->c.emplace(item);
//    }

    /**
     * @brief Get an element with defined priority
     *
     * @return PQElement
     */
    PQElement get() {
        PQElement best_item = this->top();
        this->pop();
        return best_item;
    }


    bool remove(const PQElement &value) {
        // finds if the desired value is present in the queue
        auto it = std::find(this->c.begin(), this->c.end(), value);
        if (it != this->c.end()) {
            // used to erase the element
            this->c.erase(it);
            // reorders the queue
            std::make_heap(this->c.begin(), this->c.end(), this->comp);
            return true;
        } else {
            return false;
        }
    }

    PQElement find(const PQElement &value) {
        auto it = std::find(this->c.begin(), this->c.end(), value);
        return *it;
    }

    /**
     * @brief Printing priority queue (DEBUGGING PURPOSE)
     *
     */
    inline void print_() {
        auto temp_pq = *this;
        while (!temp_pq.empty()) {
            std::cout << temp_pq.top() << std::endl;
            temp_pq.pop();
        }
    }
};