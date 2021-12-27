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

// Published topics
const std::string REAL_CoM_PATH_TOPIC("/real_com_path");
const std::string TARGET_GOAL_TOPIC("/aliengo_navigation/goal");
const std::string PREDICTED_CoM_PATH_TOPIC("/predicted_com_path");
const std::string VELOCITY_CMD_TOPIC("/aliengo/wb_controller/joy");
const std::string REAL_FEET_CONFIGURATION_MARKERS_TOPIC("/real_feet_configuration");
const std::string PREDICTED_FEET_CONFIGURATION_MARKERS_TOPIC("/predicted_feet_configuration");

// Subscribed topics
const std::string ROBOT_POSE_TOPIC("/pose");
const std::string ODOM_TOPIC("/aliengo/wb_controller/CoM");
const std::string FL_FOOT_POSE_TOPIC("/aliengo/wb_controller/lf_foot");
const std::string FR_FOOT_POSE_TOPIC("/aliengo/wb_controller/rf_foot");
const std::string RL_FOOT_POSE_TOPIC("/aliengo/wb_controller/lh_foot");
const std::string RR_FOOT_POSE_TOPIC("/aliengo/wb_controller/rh_foot");
const std::string HEIGHT_MAP_TOPIC("/elevation_mapping/elevation_map_raw");
const std::string CONTACT_FORCES_TOPIC("/aliengo/wb_controller/contact_forces");

// TF reference frames
const std::string ROBOT_REFERENCE_FRAME("trunk");
const std::string HEIGHT_MAP_REFERENCE_FRAME("world");

// Publish processed elevation map
const bool PUBLISH(true);

// Gradient threshold
const double GRADIENT_THRESHOLD(1);

// Cache size for the robot pose
const unsigned int CACHE_SIZE(10);

// Height map acquiring flag
const bool ACQUIRE_INITIAL_HEIGHT_MAP(false);

// Planner parameters
const double TIMESTAMP(0.325);
const bool SET_DIAGONAL_MOVEMENT(false);

// Footstep horizon
const unsigned int FOOTSTEP_HORIZON(2);

// Minimum distance for cell to be considered traversable
const double MIN_STAIR_DISTANCE(0.03);