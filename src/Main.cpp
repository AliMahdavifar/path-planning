/* 
 *  File: Main.cpp
 *  Username: vishnu
 *  Date: Sat Mar 30 2019
 */

#include "PathPlanner.hpp"
#include "YAMLParser.hpp"
#include "World.hpp"
#include "DStarLite.hpp"

void HAStarPlanner() {
    //    YAML::Node config = YAML::LoadFile(argv[1]);
    YAML::Node config = YAML::LoadFile("/home/vishnu/Documents/f2019/Projects/path-planning/Config.yaml");
    YAML::Node y_start;
    YAML::Node y_end;

    bool obstacles_flag = 0;

    if (config["start"])
        y_start = config["start"];
    else {
        cout << "Start not found in Config.yaml...\n";
        return;
    }

    if (config["end"])
        y_end = config["end"];
    else {
        cout << "Goal not found in Config.yaml...\n";
        return;
    }

    vector<string> obstacles;
    YAML::Node obst;
    if (config["obstacles"]) {
        obst = config["obstacles"];
        obstacles_flag = 1;
    }
    if (DEBUG)
        cout << obst.size() << endl;

    HAStar::Pose start;
    HAStar::Pose goal;

    parseGoals(y_start, start);
    parseGoals(y_end, goal);

    if (!start.isValid() || !goal.isValid()) {
        cout << "Path not achievable\n";
        return;
    }

    if (DEBUG) {
        cout << start << endl;
        cout << goal << endl;
    }

    std::vector<std::vector<int>> world(GRID_SIZE_W + 1, std::vector<int>(GRID_SIZE_H + 1, FREE_CELL));

    if (obstacles_flag)
        parseObstacles(obst, world);

    if (DEBUG) {
        for (auto vec : world) {
            for (auto elem : vec)
                cout << elem << " ";
            cout << "\n";
        }
    }

    HAStar::PathPlanner planner(world, start, goal);

    std::vector<string> moves = planner.Plan();

    if (moves.empty()) {
        cout << "Path not achievable\n";
        return;
    }

    cout << moves.size() << ",";

    size_t i;
    for (i = 0; i < moves.size() - 1; ++i)
        cout << moves[i] << ",";
    cout << moves[i] << "\n";
}

void DStarPlanner_main() {

    using DStarLite::Node;

    Node start(20, 6);
    Node goal(6, 41);
    Node tl(5, 7);
    Node br(20, 11);

    int km = 0;

    DStarLite::World world(51, 51);
    world.start = start;
    world.goal = goal;
    world.addObstacle(tl, br);

    // g values
    std::vector<std::vector<double> > g(
            world._rows,
            std::vector<double>(world._columns, world._rows * world._columns + 1));
    // rhs values
    std::vector<std::vector<double> > rhs(
            world._rows,
            std::vector<double>(world._columns, world._rows * world._columns + 1));
    // costs
    std::vector<std::vector<double> > cost(world._rows,
                                           std::vector<double>(world._columns, 1));

    DStarLite::DStarLite dsl(world,g, rhs, cost, km);


}

int main(int argc, char **argv) {
//    HAStarPlanner();
    DStarPlanner_main();
    return 0;
}