/* 
 *  File: Main.cpp
 *  Username: vishnu
 *  Date: Sat Mar 30 2019
 */

#include "PathPlanner.h"
#include "YAMLParser.h"

int main(int argc, char **argv)
{

    YAML::Node config = YAML::LoadFile(argv[1]);
    YAML::Node y_start;
    YAML::Node y_end;

    bool obstacles_flag = 0;

    if (config["start"])
        y_start = config["start"];
    else
    {
        cout << "Start not found in Config.yaml...\n";
        return 0;
    }

    if (config["end"])
        y_end = config["end"];
    else
    {
        cout << "Goal not found in Config.yaml...\n";
        return 0;
    }

    vector<string> obstacles;
    YAML::Node obst;
    if (config["obstacles"])
    {
        obst = config["obstacles"];
        obstacles_flag = 1;
    }
    if (DEBUG)
        cout << obst.size() << endl;

    Pose start;
    Pose goal;

    parseGoals(y_start, start);
    parseGoals(y_end, goal);

    if (!start.isValid() || !goal.isValid())
    {
        cout << "Path not achievable\n";
        return 0;
    }

    if (DEBUG)
    {
        cout << start << endl;
        cout << goal << endl;
    }

    std::vector<std::vector<int>> world(GRID_SIZE_W + 1, std::vector<int>(GRID_SIZE_H + 1, FREE_CELL));

    if (obstacles_flag)
        parseObstacles(obst, world);

    if (DEBUG)
    {
        for (auto vec : world)
        {
            for (auto elem : vec)
                cout << elem << " ";
            cout << "\n";
        }
    }

    PathPlanner planner(world, start, goal);

    std::vector<string> moves = planner.Plan();

    if (moves.size() == 0)
    {
        cout << "Path not achievable\n";
        return 0;
    }

    cout << moves.size() << ",";

    size_t i;
    for (i = 0; i < moves.size() - 1; ++i)
        cout << moves[i] << ",";
    cout << moves[i] << "\n";

    return 0;
}