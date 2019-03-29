#pragma once

#include "PathPlanner.h"
#include "yaml-cpp/yaml.h"

/**
 * @brief 
 * 
 * @param sn 
 * @param pose 
 */
void parseGoals(string sn, Pose &pose);

/**
 * @brief 
 * 
 * @param obst 
 * @param world 
 */
void parseObstacles(YAML::Node &obst, std::vector<std::vector<int>> &world);