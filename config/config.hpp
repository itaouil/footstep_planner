/*
 * config.hpp
 *
 *  Created on: Aug 16, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

// C++ General
#include <utility>
#include <iostream>

// Publisher topics
const std::string VELOCITY_CMD_TOPIC("/cmd_vel");

// Subscriber topics
const std::string ROBOT_POSE_TOPIC("/base_to_footprint_pose");

// TF reference frames
const std::string HEIGHT_MAP_REFERENCE_FRAME("map");
const std::string ROBOT_REFERENCE_FRAME("base_footprint");

// Cache size for the robot pose
const unsigned int ROBOT_POSE_CACHE_SIZE(10);