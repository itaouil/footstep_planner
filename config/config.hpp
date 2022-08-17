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
const std::string VELOCITY_CMD_TOPIC("/cmd_vel");
const std::string REAL_CoM_PATH_TOPIC("/real_com_path");
const std::string PREDICTED_CoM_PATH_TOPIC("/predicted_com_path");
const std::string REAL_FEET_CONFIGURATION_MARKERS_TOPIC("/real_feet_configuration");
const std::string PREDICTED_FEET_CONFIGURATION_MARKERS_TOPIC("/predicted_feet_configuration");

// Subscribed topics
const std::string HEIGHT_MAP_TOPIC("/local_gridmap");
const std::string ROBOT_POSE_TOPIC("/aliengo/ground_truth");
const std::string FEET_FORCES_TOPIC("/aliengo/contacts_state");

// TF reference frames
const std::string ROBOT_REFERENCE_FRAME("base_link");
const std::string HEIGHT_MAP_REFERENCE_FRAME("world");

// Publish processed elevation map
const bool PUBLISH(true);

// Publish CoM and feet visualization
const bool VISUALIZE(true);

// Gradient threshold
const double GRADIENT_THRESHOLD(0.7);

// Max footstep height
const float MIN_FOOT_DISTANCE(0.07);

// Max Footstep Height
const float MAX_FOOTSTEP_HEIGHT(0.07);

// Cache size for the robot pose
const unsigned int CACHE_SIZE(10);

// Height map acquiring flag
const bool ACQUIRE_INITIAL_HEIGHT_MAP(false);

// Whether to allow or disallow diagonal moves
const bool SET_DIAGONAL_MOVEMENT(false);

// Footstep horizon
const unsigned int FOOTSTEP_HORIZON(10);

// Minimum distance for cell to be considered traversable
const double MIN_STAIR_DISTANCE(0.015);

// Angle tolerance between target and robot rotation
const double ANGLE_DIFFERENCE_TOLERANCE(2);