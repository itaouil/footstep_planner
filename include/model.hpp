/*
 * model.hpp
 *
 *  Created on: Aug 24, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

// C++ general
#include <map>

// ROS general
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS messages
#include <geometry_msgs/PointStamped.h>
#include <aliengo_navigation/FootstepPrediction.h>

// Structs
#include <structs/vec2D.hpp>
#include <structs/action.hpp>
#include <structs/world2D.hpp>
#include <structs/velocityCmd.hpp>
#include <structs/displacement.hpp>
#include <structs/feetConfiguration.hpp>

// Config
#include <config.hpp>

class Model
{
public:
    /**
     * Constructor.
     */
    explicit Model(ros::NodeHandle& p_nh);

    /**
     * Destructor.
     */
    virtual ~Model();

    /**
     * Populates command to displacement map.
     */
    void populateDisplacementMap();

    /**
     * Compute next CoM (Centre of Mass) given
     * the action and velocity applied.
     *
     * @param p_velocity
     * @param p_action
     * @param p_currentCoM
     * @param p_propagateCoM
     */
    void propagateCoM(double p_velocity, const Action &p_action, const World2D &p_currentCoM, World2D &p_propagatedCoM);

    /**
     * Footsteps prediction using
     * learnt models.
     *
     * @param p_velocity
     * @param p_action
     * @param p_currentFeetConfiguration
     * @param p_predictedFeetConfiguration
     */
    void predictFeetConfiguration(double p_velocity,
                                  const Action &p_action,
                                  const FeetConfiguration &p_currentFeetConfiguration,
                                  FeetConfiguration &p_predictedFeetConfiguration);
private:
    //! ROS node handle
    ros::NodeHandle m_nh;

    //! ROS height map service request
    ros::ServiceClient m_footstepPredictionServiceClient;

    //! Dictionary containing displacements for each velocity
    std::map<VelocityCmd, Displacement> m_displacementMap;
};