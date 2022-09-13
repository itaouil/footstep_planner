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
const std::string VELOCITY_CMD_TOPIC("/aliengo_bridge/twist_cmd");
const std::string PREDICTED_CoM_PATH_TOPIC("/predicted_com_path");
const std::string REAL_FEET_CONFIGURATION_MARKERS_TOPIC("/real_feet_configuration");
const std::string PREDICTED_FEET_CONFIGURATION_MARKERS_TOPIC("/predicted_feet_configuration");

// Subscribed topics
const std::string HEIGHT_MAP_TOPIC("/local_gridmap");
const std::string ROBOT_POSE_TOPIC("/t265/odom/sample");
const std::string HIGH_STATE_SUBSCRIBER("/aliengo_bridge/high_state");

// TF reference frames
const std::string ROBOT_REFERENCE_FRAME("base");
const std::string LF_FOOT_REFERENCE_FRAME("FL_foot");
const std::string RF_FOOT_REFERENCE_FRAME("FR_foot");
const std::string LH_FOOT_REFERENCE_FRAME("RL_foot");
const std::string RH_FOOT_REFERENCE_FRAME("RR_foot");
const std::string HEIGHT_MAP_REFERENCE_FRAME("t265_odom_frame");

// Publish processed elevation map
const bool PUBLISH(true);

// Publish CoM and feet visualization
const bool VISUALIZE(false);

// Gradient threshold
const double GRADIENT_THRESHOLD(0.2);

// Max footstep height
const float MIN_FOOT_DISTANCE(0.04);

// Max Footstep Height
const float MAX_FOOTSTEP_HEIGHT(0.14);

// Cache size for the robot pose
const unsigned int CACHE_SIZE(10);

// Height map acquiring flag
const bool ACQUIRE_INITIAL_HEIGHT_MAP(false);

// Whether to allow or disallow diagonal moves
const bool SET_DIAGONAL_MOVEMENT(false);

// Footstep horizon
const unsigned int FOOTSTEP_HORIZON(7);

// Minimum distance for cell to be considered traversable
const float MIN_STAIR_DISTANCE(0.08);

// Angle tolerance between target and robot rotation
const float ANGLE_DIFFERENCE_TOLERANCE(2);

// Out of contact force
const float OUT_OF_CONTACT_FORCE(10.0);

// Back in contact force
const float BACK_IN_CONTACT_FORCE(30.0);