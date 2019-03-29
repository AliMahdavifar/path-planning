#pragma once

#include <iostream>
#include <cmath>

// Data Structures
#include <vector>
#include <map>
#include <queue>
using namespace std;

#define GRID_SIZE_W 13
#define GRID_SIZE_H 6
#define FREE_CELL 0

#define DEBUG 0

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
    Pose parent;
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
     * @param parent 
     * @param f 
     * @param g 
     * @param h 
     * @param command 
     */
    Node(const Pose &pose, const Pose &parent, const double &f, const double &g, const double &h, const string &command)
        : pose(pose), parent(parent), f(f), g(g), h(h), command(command)
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
    PathPlanner(/* args */);
    
    /**
     * @brief Construct a new Path Planner object
     * 
     * @param world 
     * @param start 
     * @param goal 
     */
    PathPlanner(const vector<vector<int>> &world, const Pose &start, const Pose &goal);
    
    /**
     * @brief 
     * 
     * @param current_state 
     * @param command 
     * @return Pose 
     */
    Pose NextState(const Pose &current_state, const Command &command);
    
    /**
     * @brief 
     * 
     * @param current_node 
     * @return vector<Node> 
     */
    vector<Node> Expand(const Node &current_node);
    
    /**
     * @brief 
     * 
     * @param world 
     * @param start 
     * @param goal 
     * @return vector<string> 
     */
    vector<string> Plan(const vector<vector<int>> &world, const Pose &start, const Pose &goal);
    
    /**
     * @brief 
     * 
     * @param p1 
     * @param goal 
     * @return double 
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

template <typename PQElement>
struct PriorityQueue
{
    /**
     * @brief 
     * 
     */

    priority_queue<PQElement, vector<PQElement>,
                   greater<PQElement>>
        elements;

    /**
     * @brief 
     * 
     * @return true 
     * @return false 
     */
    inline bool empty() const
    {
        return elements.empty();
    }

    /**
     * @brief 
     * 
     * @param item 
     */
    inline void push(PQElement item)
    {
        elements.emplace(item);
    }

    /**
     * @brief 
     * 
     * @return PQElement 
     */
    PQElement get()
    {
        PQElement best_item = elements.top();
        elements.pop();
        return best_item;
    }
    /**
     * @brief 
     * 
     */
    inline void print_()
    {
        auto temp_pq = elements;
        while (!temp_pq.empty())
        {
            cout << temp_pq.top() << endl;
            temp_pq.pop();
        }
    }
};