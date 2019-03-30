/* 
 *  File: YAMLParser.h
 *  Username: vishnu
 *  Date: Sat Mar 30 2019
 */

#pragma once

#include "PathPlanner.h"
#include "yaml-cpp/yaml.h"

/**
 * @brief Sets pose value read from YAML
 * 
 * @param sn 
 * @param pose 
 */
void parseGoals(YAML::Node &sn, Pose &pose);

/**
 * @brief Inserts obstcles in the world defined in YAML
 * 
 * @param obst 
 * @param world 
 */
void parseObstacles(YAML::Node &obst, std::vector<std::vector<int>> &world);