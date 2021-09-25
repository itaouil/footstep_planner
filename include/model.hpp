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
#include <visualization_msgs/MarkerArray.h>
#include <aliengo_navigation/FootstepPrediction.h>

// Structs
#include <structs/node.hpp>
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
     * Transform absolute footstep sizes
     * from CoM reference frame to map frame.
     *
     * @param p_rotation
     * @param p_predictionsCoMFrame
     * @param p_predictionsMapFrame
     */
    void transformPredictionsToMapFrame(const tf2::Quaternion &p_rotation,
                                        const std::vector<float> &p_predictionsCoMFrame,
                                        std::vector<double> &p_predictionsMapFrame);

    /**
     * Compute new (map) feet configuration
     * using the predicted footsteps.
     *
     * @param l_predictionsMapFrame
     * @param p_currentFeetConfiguration
     * @param p_newFeetConfiguration
     */
    void computeMapFeetConfiguration(const std::vector<double> &l_predictionsMapFrame,
                                     const FeetConfiguration &p_currentFeetConfiguration,
                                     FeetConfiguration &p_newFeetConfiguration);

    /**
     * Compute new CoM in world coordinates.
     *
     * @param p_angularVelocity
     * @param p_predictedCoMDisplacementX
     * @param p_predictedCoMDisplacementY
     * @param p_currentWorldCoordinatesCoM
     * @param p_newWorldCoordinatesCoM
     */
    void computeNewCoM(double p_angularVelocity,
                       double p_predictedCoMDisplacementX,
                       double p_predictedCoMDisplacementY,
                       const World2D &p_currentWorldCoordinatesCoM,
                       World2D &p_newWorldCoordinatesCoM);

    /**
     * Compute new (CoM) feet configuration
     * using the newly computed map feet
     * configuration and CoM.
     *
     * @param l_relativeStepPredictions
     * @param p_currentFeetConfiguration
     * @param p_newFeetConfiguration
     */
    void computeNewCoMFeetConfiguration(const std::vector<double> &l_relativeStepPredictions,
                                        const FeetConfiguration &p_currentFeetConfiguration,
                                        FeetConfiguration &p_newFeetConfiguration);

    /**
     * Predicts new feet configuration using
     * the learnt models and extracts new CoM
     * from them.
     *
     * @param p_velocity
     * @param p_action
     * @param p_currentWorldCoordinatesCoM
     * @param p_currentFeetConfiguration
     * @param p_newFeetConfiguration
     * @param p_newWorldCoordinatesCoM
     */
    void predictNewConfiguration(double p_velocity,
                                 const Action &p_action,
                                 const World2D &p_currentWorldCoordinatesCoM,
                                 const FeetConfiguration &p_currentFeetConfiguration,
                                 FeetConfiguration &p_newFeetConfiguration,
                                 World2D &p_newWorldCoordinatesCoM);
private:
    //! ROS node handle
    ros::NodeHandle m_nh;

    //! TF2 buffer
    tf2_ros::Buffer m_buffer;

    //! TF2 listener
    tf2_ros::TransformListener m_listener;

    ros::Publisher m_feetConfigurationPublisher;

    //! ROS height map service request
    ros::ServiceClient m_footstepPredictionServiceClient;
};