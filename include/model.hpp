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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

// Structs
#include <structs/node.hpp>
#include <structs/vec2D.hpp>
#include <structs/action.hpp>
#include <structs/world3D.hpp>
#include <structs/velocityCmd.hpp>
#include <structs/displacement.hpp>
#include <structs/feetConfiguration.hpp>

// Eigen
#include <Eigen/Dense>

// Config
#include <config.hpp>

class Model {
public:
    /**
     * Constructor.
     */
    explicit Model(ros::NodeHandle &p_nh);

    /**
     * Destructor.
     */
    virtual ~Model();

    /**
     * Populates the respective model
     * coefficients vectors required for
     * the prediction process of continuous
     * velocity commands.
     */
    void setModelsCoefficients();

    /**
     * Compute CoM and feet displacements
     * predictions when a discontinuous velocity
     * is used (i.e. acceleration).
     *
     * @param p_previousVelocityX
     * @param p_previousVelocityY
     * @param p_previousAngularVelocity
     * @param p_nextVelocityX
     * @param p_nextVelocityY
     * @param p_nextAngularVelocity
     * @param p_odomVelocityState
     * @param p_currentFeetConfiguration
     * @param p_predictions
     */
    void prediction(double p_previousVelocityX,
                    double p_previousVelocityY,
                    double p_previousAngularVelocity,
                    double p_nextVelocityX,
                    double p_nextVelocityY,
                    double p_nextAngularVelocity,
                    const geometry_msgs::Twist &p_odomVelocityState,
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
                       const World3D &p_currentWorldCoordinatesCoM,
                       World3D &p_newWorldCoordinatesCoM);

    /**
     * Compute new (CoM) feet configuration
     * using the newly computed map feet
     * configuration and CoM.
     *
     * @param p_newWorldCoordinatesCoM
     * @param p_predictions
     * @param p_currentFeetConfiguration
     * @param p_newFeetConfiguration
     */
    void computeNewFeetConfiguration(const World3D &p_newWorldCoordinatesCoM,
                                     const std::vector<double> &p_predictions,
                                     const FeetConfiguration &p_currentFeetConfiguration,
                                     FeetConfiguration &p_newFeetConfiguration);

    /**
     * Predicts new feet configuration using
     * the learnt models and extracts new CoM
     * from them.
     *
     * @param p_accelerating
     * @param p_previousVelocity
     * @param p_currentVelocity
     * @param p_action
     * @param p_velocityState
     * @param p_currentWorldCoordinatesCoM
     * @param p_currentFeetConfiguration
     * @param p_newFeetConfiguration
     * @param p_newWorldCoordinatesCoM
     */
    void predictNextState(bool p_accelerating,
                          double p_previousVelocity,
                          double p_currentVelocity,
                          const Action &p_action,
                          const geometry_msgs::Twist &p_odomVelocityState,
                          const World3D &p_currentWorldCoordinatesCoM,
                          const FeetConfiguration &p_currentFeetConfiguration,
                          FeetConfiguration &p_newFeetConfiguration,
                          World3D &p_newWorldCoordinatesCoM);

private:
    //! ROS node handle
    ros::NodeHandle m_nh;

    //! TF2 buffer and listener
    tf2_ros::Buffer m_buffer;
    tf2_ros::TransformListener m_listener;

    //! Publisher for feet poses (debugging)
    ros::Publisher m_feetConfigurationPublisher;

    //! CoM models' coefficients
    Eigen::RowVectorXd m_fr_rl_com_x;
    Eigen::RowVectorXd m_fr_rl_com_y;
    Eigen::RowVectorXd m_fl_rr_com_x;
    Eigen::RowVectorXd m_fl_rr_com_y;

    //! FL models' coefficients
    Eigen::RowVectorXd m_fl_swinging_x;
    Eigen::RowVectorXd m_fl_swinging_y;

    //! FR models' coefficients
    Eigen::RowVectorXd m_fr_swinging_x;
    Eigen::RowVectorXd m_fr_swinging_y;

    //! RL models' coefficients
    Eigen::RowVectorXd m_rl_swinging_x;
    Eigen::RowVectorXd m_rl_swinging_y;

    //! RR models' coefficients
    Eigen::RowVectorXd m_rr_swinging_x;
    Eigen::RowVectorXd m_rr_swinging_y;
};