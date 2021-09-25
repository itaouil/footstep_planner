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
const std::string PATH_TOPIC("/path");
const std::string VELOCITY_CMD_TOPIC("/cmd_vel");
const std::string FEET_CONFIGURATION_MARKERS_TOPIC("/feet_configuration");

// Subscriber topics
const std::string ROBOT_POSE_TOPIC("/base_to_footprint_pose");
const std::string FEET_CONFIGURATION_TOPIC("/foot");

// Services
const std::string FOOTSTEP_PREDICTION_SERVICE_TOPIC("/footstep_prediction");
const std::string HEIGHT_MAP_SERVICE_TOPIC("/elevation_mapping/get_raw_submap");

// TF reference frames
const std::string HEIGHT_MAP_REFERENCE_FRAME("map");
const std::string ROBOT_REFERENCE_FRAME("base_footprint");

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