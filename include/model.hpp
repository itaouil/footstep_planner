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
#include <iostream>

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

// Eigen
#include <Eigen/Dense>

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
     * Populates the respective model
     * coefficients vectors required for
     * the prediction process.
     */
    void setModelsCoefficients();

    /**
     * Compute CoM and feet displacements
     * predictions using the learnt model
     * coefficients.
     *
     * @param p_velocityX
     * @param p_velocityY
     * @param p_angularVelocity
     * @param p_currentFeetConfiguration
     * @param p_predictions
     */
    void computePredictedDisplacements(double p_velocityX,
                                       double p_velocityY,
                                       double p_angularVelocity,
                                       const FeetConfiguration &p_currentFeetConfiguration,
                                       std::vector<double> &p_predictions);

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

    //! CoM models' coefficients
    Eigen::RowVectorXd m_fr_rl_com_x;
    Eigen::RowVectorXd m_fr_rl_com_y;
    Eigen::RowVectorXd m_fl_rr_com_x;
    Eigen::RowVectorXd m_fl_rr_com_y;

    //! FL models' coefficients
    Eigen::RowVectorXd m_fl_support_x;
    Eigen::RowVectorXd m_fl_support_y;
    Eigen::RowVectorXd m_fl_swinging_x;
    Eigen::RowVectorXd m_fl_swinging_y;

    //! FR models' coefficients
    Eigen::RowVectorXd m_fr_support_x;
    Eigen::RowVectorXd m_fr_support_y;
    Eigen::RowVectorXd m_fr_swinging_x;
    Eigen::RowVectorXd m_fr_swinging_y;

    //! RL models' coefficients
    Eigen::RowVectorXd m_rl_support_x;
    Eigen::RowVectorXd m_rl_support_y;
    Eigen::RowVectorXd m_rl_swinging_x;
    Eigen::RowVectorXd m_rl_swinging_y;

    //! RR models' coefficients
    Eigen::RowVectorXd m_rr_support_x;
    Eigen::RowVectorXd m_rr_support_y;
    Eigen::RowVectorXd m_rr_swinging_x;
    Eigen::RowVectorXd m_rr_swinging_y;
};