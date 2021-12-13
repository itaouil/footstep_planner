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
const std::string ROBOT_POSE_TOPIC("/pose");
const std::string FL_FOOT_POSE_TOPIC("/aliengo/wb_controller/lf_foot");
const std::string FR_FOOT_POSE_TOPIC("/aliengo/wb_controller/rf_foot");
const std::string RL_FOOT_POSE_TOPIC("/aliengo/wb_controller/lh_foot");
const std::string RR_FOOT_POSE_TOPIC("/aliengo/wb_controller/rh_foot");
const std::string HEIGHT_MAP_TOPIC("/elevation_mapping/elevation_map_raw");

// TF reference frames
const std::string HEIGHT_MAP_REFERENCE_FRAME("world");
const std::string ROBOT_REFERENCE_FRAME("trunk");

// Publish processed elevation map
const bool PUBLISH(true);

// Gradient threshold
const double GRADIENT_THRESHOLD(1.5);

// Cache size for the robot pose
const unsigned int CACHE_SIZE(10);

// Height map acquiring flag
const bool ACQUIRE_INITIAL_HEIGHT_MAP(false);

// Planner parameters
const double TIMESTAMP(0.325);
const bool SET_DIAGONAL_MOVEMENT(false);

// Footstep horizon
const unsigned int FOOTSTEP_HORIZON(2);