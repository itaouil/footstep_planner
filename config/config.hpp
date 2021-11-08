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
const std::string TARGET_PATH_TOPIC("/target_path");
const std::string REAL_PATH_TOPIC("/performed_path");
const std::string REAL_FEET_CONFIGURATION_MARKERS_TOPIC("/real_feet_configuration");
const std::string TARGET_FEET_CONFIGURATION_MARKERS_TOPIC("/target_feet_configuration");

// Subscriber topics
const std::string ODOM_TOPIC("/odom");
const std::string ROBOT_POSE_TOPIC("/base_to_footprint_pose");

// Services
const std::string HEIGHT_MAP_TOPIC("/elevation_mapping/elevation_map_raw");
const std::string HEIGHT_MAP_SERVICE_TOPIC("/elevation_mapping/elevation_map_raw");

// TF reference frames
const std::string HEIGHT_MAP_REFERENCE_FRAME("odom");
const std::string ROBOT_REFERENCE_FRAME("base_footprint");

// Whether to publish data
const bool PUBLISH(true);

// Gradient threshold
const unsigned short GRADIENT_THRESHOLD(90);

// Cache size for the robot pose
const unsigned int CACHE_SIZE(10);

// Grid map service parameters
const unsigned int HEIGHT_MAP_LENGTH_X(7);
const unsigned int HEIGHT_MAP_LENGTH_Y(7);

// Height map acquiring flag
const bool ACQUIRE_INITIAL_HEIGHT_MAP(false);

// Planner parameters
const double TIMESTAMP(0.325);
const bool SET_DIAGONAL_MOVEMENT(false);
const double HEIGHT_MAP_RESOLUTION(0.04);
const int HEIGHT_MAP_GRID_SIZE_X(static_cast<int>(HEIGHT_MAP_LENGTH_X / HEIGHT_MAP_RESOLUTION) + 1);
const int HEIGHT_MAP_GRID_SIZE_Y(static_cast<int>(HEIGHT_MAP_LENGTH_Y / HEIGHT_MAP_RESOLUTION) + 1);