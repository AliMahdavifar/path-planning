//
// Created by vishnu on 9/28/19.
//
#include <gtest/gtest.h>
#include "DStarLite.hpp"
#include "Node.hpp"

TEST(SuiteName, TestName) {

    using DStarLite::Node;

    Node node(1,2);
    EXPECT_EQ(node._x,1);
    EXPECT_EQ(node._y, 2);
}
