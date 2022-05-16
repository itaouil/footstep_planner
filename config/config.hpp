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
const std::string PREDICTED_CoM_PATH_TOPIC("/predicted_com_path");
const std::string VELOCITY_CMD_TOPIC("/aliengo_bridge/cmd");
const std::string REAL_FEET_CONFIGURATION_MARKERS_TOPIC("/real_feet_configuration");
const std::string PREDICTED_FEET_CONFIGURATION_MARKERS_TOPIC("/predicted_feet_configuration");

// Subscribed topics
const std::string ROBOT_POSE_TOPIC("/t265/odom/sample");
const std::string HIGH_STATE_TOPIC("/aliengo_bridge/high_state");
const std::string HEIGHT_MAP_TOPIC("/elevation_mapping/elevation_map_raw");

// TF reference frames
const std::string ROBOT_REFERENCE_FRAME("base");
const std::string HEIGHT_MAP_REFERENCE_FRAME("t265_odom_frame");

// Publish processed elevation map
const bool PUBLISH(true);

// Gradient threshold
const double GRADIENT_THRESHOLD(0.7);

// Max footstep height
const float MIN_FOOT_DISTANCE(0.07);

// Max Footstep Height
const float MAX_FOOTSTEP_HEIGHT(0.07);

// Cache size for the robot pose
const unsigned int CACHE_SIZE(200);

// Height map acquiring flag
const bool ACQUIRE_INITIAL_HEIGHT_MAP(false);

// Time taken for a footstep (approx)
const double TIMESTAMP(0.33);

// Whether to allow or disallow diagonal moves
const bool SET_DIAGONAL_MOVEMENT(false);

// Footstep horizon
const unsigned int FOOTSTEP_HORIZON(5);

// Minimum distance for cell to be considered traversable
const double MIN_STAIR_DISTANCE(0.01);

// Angle tolerance between target and robot rotation
const double ANGLE_DIFFERENCE_TOLERANCE(2);