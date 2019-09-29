/* 
 *  File: PathPlanner.h
 *  Username: vishnu
 *  Date: Sat Mar 30 2019
 */

#pragma once

#include <iostream>
#include <cmath>

// Data Structures
#include <vector>
#include <map>
#include <queue>

#include "PriorityQueue.hpp"
using namespace std;

#define GRID_SIZE_W 13
#define GRID_SIZE_H 6
#define FREE_CELL 0

#define DEBUG 0

namespace HAStar
{
/**
 * @brief Directions
 *  F - Forward
 *  B - Reverse
 */
enum Direction
{
    F = 1,
    B = -1
};

/**
 * @brief Steering direction
 *  R - Right
 *  S - Straight
 *  L - Left
 */
enum Steering
{
    R = -1,
    S = 0,
    L = 1
};


struct Pose
{
    /**
     * @brief STRUCT Pose
     *  This defines the pose of the agent.
     *  Here, theta takes only four directions : North - 0(90 degrees), WEST - 1(180 degrees), SOUTH - 2(270 degree), EAST - 3 (0 degrees)
     */
    int x;
    int y;
    int theta;
    /**
     * @brief Construct a new Pose object
     *
     */
    Pose()
    {
    }

    /**
     * @brief Construct a new Pose object
     *
     * @param x
     * @param y
     * @param theta
     */
    Pose(const int &x, const int &y, const int &theta) : x(x), y(y), theta(theta)
    {
    }

    /**
     * @brief Destroy the Pose object
     *
     */
    virtual ~Pose()
    {

    }
    /**
     * @brief Checks if the pose is valid in the given world
     *
     * @return true
     * @return false
     */
    bool isValid()
    {
        return !(x < 0 || x > GRID_SIZE_W || y < 0 || y > GRID_SIZE_H);
    }

    /**
     * @brief Overloading output stream (DEBUGGING PURPOSE)
     *
     * @param os
     * @param ps
     * @return ostream&
     */
    friend ostream &operator<<(ostream &os, const Pose &ps)
    {
        os << "X: " << ps.x << " Y: " << ps.y << " Theta: " << ps.theta;
        return os;
    }

    /**
     * @brief Overloading '==' operator
     *
     * @param lhs
     * @param rhs
     * @return true
     * @return false
     */
    friend bool operator==(const Pose &lhs, const Pose &rhs)
    {
        return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.theta == rhs.theta);
    }
};

struct Node
{
    /**
     * @brief STRUCT Node
     *
     * Implements a node that holds the costs, poses, and corresponding commands
     *
     */
    Pose pose;
    // Pose parent;
    double f;
    double g;
    double h;

    // it takes the value from one of the six possibilities from motion model
    // FS - Forward, Straight
    // FR - Forward, Right
    // FL - Forward, Left
    // BS - Reverse, Straight
    // BR - Reverse, Right
    // BL - Reverse, Left
    string command;

    /**
     * @brief Construct a new Node object
     *
     */
    Node()
    {
    }
    /**
     * @brief Construct a new Node object
     *
     * @param pose
     * @param f
     * @param g
     * @param h
     * @param command
     */
    Node(const Pose &pose, const double &f, const double &g, const double &h, const string &command)
        : pose(pose), f(f), g(g), h(h), command(command)
    {
    }

    /**
     * @brief Construct a new Node object
     *
     * @param pose
     * @param f
     * @param g
     * @param h
     */
    Node(const Pose &pose, const double &f, const double &g, const double &h)
        : pose(pose), f(f), g(g), h(h)
    {
    }

    /**
     * @brief Destroy the Node object
     *
     */
    virtual ~Node()
    {

    }

    /**
     * @brief Overloading '<' Operator
     *
     * @param lhs
     * @param rhs
     * @return true
     * @return false
     */
    friend bool operator<(const Node &lhs, const Node &rhs)
    {
        if (lhs.f == rhs.f)
            return lhs.h < rhs.h;
        return lhs.f < rhs.f;
    }

    /**
     * @brief Overloading '>' Operator
     *
     * @param lhs
     * @param rhs
     * @return true
     * @return false
     */
    friend bool operator>(const Node &lhs, const Node &rhs)
    {
        if (lhs.f == rhs.f)
            return lhs.h > rhs.h;
        return lhs.f > rhs.f;
    }

    /**
     * @brief Overloading output stream (DEBUGGING PURPOSE)
     *
     * @param os
     * @param node
     * @return ostream&
     */
    friend ostream &operator<<(ostream &os, const Node &node)
    {
        os << "Pose: " << node.pose << " f: " << node.f << " g: " << node.g << " h: " << node.h << " Command: " << node.command;
        return os;
    }
};


class PathPlanner
{
    /**
     * @brief Class PathPlanner
     *  Implements a simple, highly discretized version of Hybrid A*
     */
    typedef pair<int, int> location;
    typedef pair<int, int> Command;
    bool initialize_flag = 0;

  public:
    /* Six possible motions - FS, FR, FL, BS, BR, BL */
    map<string, pair<int, int>> m_MotionModel;
    vector<vector<int>> world;
    Pose start;
    Pose goal;

    /**
     * @brief Construct a new Path Planner object
     *
     */
    PathPlanner();

    /**
     * @brief Construct a new Path Planner object
     *
     * @param world Grid defined as a 2d vector 0 - FREECELL, 1 - OCCUPIED
     * @param start Pose(X, Y, Theta)
     * @param goal Pose(X,Y,Theta)
     */
    PathPlanner(const vector<vector<int>> &world, const Pose &start, const Pose &goal);

    /**
     * @brief Takes current state and command to compute next state using motion model
     *
     * @param current_state Pose(X,Y,Theta)
     * @param command Command pair<direction, steering>
     * @return Pose  Pose(X,Y,Theta)
     */
    Pose NextState(const Pose &current_state, const Command &command);

    /**
     * @brief Takes current node and computes all possible, valied nodes to explore
     *
     * @param current_node Node
     * @return vector<Node>
     */
    vector<Node> Expand(const Node &current_node);

    /**
     * @brief Function implements A* logic to compute optimal set of actions
     *
     * @return vector<string>
     */
    vector<string> Plan();

    /**
     * @brief Computes Euclidean distance between poses
     *
     * @param p1 Pose(X,Y,Theta)
     * @param goal Pose(X,Y,Theta)
     * @return double distance
     */
    double Heuristic(const Pose &p1, const Pose &goal);

    /**
     * @brief Get the Motion Model object
     *
     */
    void GetMotionModel();

    /**
     * @brief Destroy the Path Planner object
     *
     */
    virtual ~PathPlanner();
};
}