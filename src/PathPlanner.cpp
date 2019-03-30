/* 
 *  File: PathPlanner.cpp
 *  Username: vishnu
 *  Date: Sat Mar 30 2019
 */

#include "PathPlanner.h"

void PathPlanner::GetMotionModel()
{
    m_MotionModel["FR"] = {F, R};
    m_MotionModel["FS"] = {F, S};
    m_MotionModel["FL"] = {F, L};
    m_MotionModel["BR"] = {B, R};
    m_MotionModel["BS"] = {B, S};
    m_MotionModel["BL"] = {B, L};
}

PathPlanner::PathPlanner()
{
    // world, start, and goal is not set
    GetMotionModel();
}

PathPlanner::PathPlanner(const vector<vector<int>> &world, const Pose &start, const Pose &goal) : world(world), start(start), goal(goal)
{
    // indicates that the world, start, and goal are initialized for the members to use
    initialize_flag = 1;
    GetMotionModel();
}

PathPlanner::~PathPlanner()
{
}

double PathPlanner::Heuristic(const Pose &p1, const Pose &goal)
{
    return sqrt(pow((p1.x - goal.x), 2) + pow((p1.y - goal.y), 2) + pow((p1.theta - goal.theta), 2));
}

Pose PathPlanner::NextState(const Pose &current_state, const Command &command)
{
    // Convert encoded direction to angle
    float theta = float(current_state.theta + 1) * (M_PI / 2.0);

    // 0<=theta<=2*PI
    while (theta > 2 * M_PI)
        theta -= 2 * (M_PI);

    // System dynamics
    int x = current_state.x + command.first * int(cos(theta));
    int y = current_state.y - command.first * int(sin(theta));

    if (command.second != 0)
    {
        theta = theta + command.first * command.second * M_PI / 2;
        x = x + command.first * int(cos(theta));
        y = y - command.first * int(sin(theta));
    }

    // convert angle back to direction
    int op = int(round(theta / (M_PI / 2) - 1));

    if (op < 0)
        op += 4;

    Pose ret(x, y, op);

    return ret;
}

vector<Node> PathPlanner::Expand(const Node &node)
{
    double g = node.g;
    double h = node.h;

    Pose pose = node.pose;
    vector<Node> next_nodes;

    for (auto move : m_MotionModel)
    {
        Pose current_state = NextState(pose, move.second);

        // Check if the state is valid and free
        if (current_state.isValid() && !(world[current_state.x][current_state.y]))
        {
            Node current_node;
            current_node.pose = current_state;
            current_node.h = Heuristic(current_state, goal);
            current_node.g = g + Heuristic(pose, current_state);
            current_node.f = g + h;
            current_node.command = move.first;
            next_nodes.push_back(current_node);
        }
    }
    return next_nodes;
}

vector<string> PathPlanner::Plan()
{
    if (!initialize_flag)
    {
        cout << "Initialize the environment by setting world, start and goal values\n";
        return vector<string>{};
    }

    if (DEBUG)
    {
        cout << start << endl;
        cout << goal << endl;
    }

    vector<string> ret;

    // 3D vector to track visited node
    vector<vector<vector<int>>> closed(4, vector<vector<int>>(world.size(), vector<int>(world[0].size())));

    // 3D vector to track parent nodes
    vector<vector<vector<Node>>> came_from(4, vector<vector<Node>>(world.size(), vector<Node>(world[0].size())));


    // Initialize the frontier
    Node state;

    state.pose = start;

    double g = 0;
    double h = Heuristic(start, goal);
    double f = g + h;

    state.f = f;
    state.g = g;
    state.h = h;
    closed[start.theta][state.pose.x][state.pose.y] = 1;
    came_from[start.theta][state.pose.x][state.pose.y] = state;


    int total_closed = 1;

    PriorityQueue<Node> frontier;
    frontier.push(state);

    while (!frontier.empty())
    {
        if (DEBUG)
        {
            cout << "exploring...\n";
            cout << "Frontier:" << endl;
            frontier.print_();
            cout << "Frontier end...\n";
        }

        // Frontier, by default, gives the node with the lowest cost
        Node next = frontier.get();

        // Build the result
        if (next.pose == goal)
        {

            ret.insert(ret.begin(), next.command);

            if (DEBUG)
                cout << next.command << " " << next.pose << endl;
            
            Node current_node = came_from[goal.theta][goal.x][goal.y];
            while (!(current_node.pose == start))
            {
            
                if (DEBUG)
                    cout << current_node.command << " " << current_node.pose << endl;
            
                ret.insert(ret.begin(), current_node.command);
                current_node = came_from[current_node.pose.theta][current_node.pose.x][current_node.pose.y];
            }

            if (DEBUG)
                cout << "Found path to goal..." << endl;
            
            return ret;
        }

        if (DEBUG)
            cout << "EXPANDING :: " << next << endl;

        vector<Node> next_nodes = Expand(next);

        if (DEBUG)
        {
            cout << "NextNODES:: \n";
            for (auto x : next_nodes)
            {
                cout << x << "\n";
            }
        }

        for (auto node : next_nodes)
        {

            Pose current_pose = node.pose;

            // if not explored the currently generated pose
            if (closed[current_pose.theta][current_pose.x][current_pose.y] == 0)
            {
                // make a node and add it to the frontier
                Node current_node;

                current_node.f = node.f;
                current_node.g = node.g;
                current_node.h = node.h;
                current_node.command = node.command;
                current_node.pose = current_pose;
                frontier.push(current_node);

                // mark as visited
                closed[current_pose.theta][current_pose.x][current_pose.y] = 1;
                came_from[current_pose.theta][current_pose.x][current_pose.y] = next;

                // just to track the number of nodes explored
                total_closed++;
            }
        }
    }

    return ret;
}