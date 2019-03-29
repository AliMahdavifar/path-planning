#include "YAMLParser.h"

void parseGoals(string sn, Pose &pose)
{

    std::istringstream str_buf{sn};
    int x;
    std::vector<int> v;
    
    while (str_buf >> x)
    {
        v.push_back(x);
        // If the next char in input is a comma, extract it. std::ws discards whitespace
        if ((str_buf >> std::ws).peek() == ',')
            str_buf.ignore();
    }

    pose.x = v[0];
    pose.y = v[1];
    pose.theta = v[2];
}

void parseObstacles(YAML::Node &obst, std::vector<std::vector<int>> &world)
{
    for (YAML::iterator it = obst.begin(); it != obst.end(); ++it)
    {
        int x, y;
        string loc = (*it).as<string>();
        std::istringstream str_buf{loc};
        str_buf >> x;
        if ((str_buf >> std::ws).peek() == ',')
            str_buf.ignore();
        str_buf >> y;
        world[x][y] = 1;
    }
}