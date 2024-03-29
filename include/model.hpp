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
     * velocity commands for the real robot;
     */
    void setModelsCoefficientsReal();

    /**
     * Populates the respective model
     * coefficients vectors required for
     * the prediction process of continuous
     * velocity commands for the simulated robot;
     */
    void setModelsCoefficientsSimulation();

    /**
     * Quaternion to rotation matrix.
     * 
     * @param q 
     * @return Eigen::Matrix3d 
     */
    inline Eigen::Matrix3d quatToRotMat(const Eigen::Quaterniond & q) {
        Eigen::Matrix3d R;
        R(0, 0) = -1.0 + 2.0 * (q.w() * q.w()) + 2.0 * (q.x() * q.x());
        R(1, 1) = -1.0 + 2.0 * (q.w() * q.w()) + 2.0 * (q.y() * q.y());
        R(2, 2) = -1.0 + 2.0 * (q.w() * q.w()) + 2.0 * (q.z() * q.z());
        R(0, 1) = 2.0 * (q.x() * q.y() + q.w() * q.z());
        R(0, 2) = 2.0 * (q.x() * q.z() - q.w() * q.y());
        R(1, 0) = 2.0 * (q.x() * q.y() - q.w() * q.z());
        R(1, 2) = 2.0 * (q.y() * q.z() + q.w() * q.x());
        R(2, 0) = 2.0 * (q.x() * q.z() + q.w() * q.y());
        R(2, 1) = 2.0 * (q.y() * q.z() - q.w() * q.x());

        return R;
    }

    /**
     * Motion prediction.
     *
     * @param p_plannedHorizon
     * @param p_previousVelocityX
     * @param p_previousVelocityY
     * @param p_previousAngularVelocity
     * @param p_nextVelocityX
     * @param p_nextVelocityY
     * @param p_nextAngularVelocity
     * @param p_previousBaseVelocity
     * @param p_currentBaseVelocity
     * @param p_currentFeetConfiguration
     * @param p_predictions
     */
     void motionPrediction(const uint &p_plannedHorizon,
                           const double &p_previousVelocityX,
                           const double &p_previousVelocityY,
                           const double &p_previousAngularVelocity,
                           const double &p_nextVelocityX,
                           const double &p_nextVelocityY,
                           const double &p_nextAngularVelocity,
                           const double &p_previousBaseVelocity,
                           const double &p_currentBaseVelocity,
                           const FeetConfiguration &p_currentFeetConfiguration,
                           std::vector<double> &p_predictions);

    /**
     * Predict next velocity.
     *
     * @param p_previousVelocityX
     * @param p_previousVelocityY
     * @param p_previousAngularVelocity
     * @param p_nextVelocityX
     * @param p_nextVelocityY
     * @param p_nextAngularVelocity
     * @param p_previousCoMVelocity
     * @param p_currentCoMVelocity
     */
    double velocityPrediction(const double &p_previousVelocityX,
                              const double &p_previousVelocityY,
                              const double &p_previousAngularVelocity,
                              const double &p_nextVelocityX,
                              const double &p_nextVelocityY,
                              const double &p_nextAngularVelocity,
                              const double &p_previousCoMVelocity,
                              const double &p_currentCoMVelocity);

    /**
     * Rotate predictions according
     * to robot's rotation.
     * 
     * @param l_predictions 
     * @param l_odometry 
     */
    void rotatePredictions(std::vector<double> &l_predictions, const World3D &l_odometry);

    /**
     * Compute new CoM in world coordinates.
     *
     * @param p_predictedCoMDisplacementX
     * @param p_predictedCoMDisplacementY
     * @param p_predictedCoMDisplacementTheta
     * @param p_currentCoMVelocity
     * @param p_predictedCoMVelocity
     * @param p_currentWorldCoordinatesCoM
     * @param p_newWorldCoordinatesCoM
     */
    void computeNewCoM(const double &p_predictedCoMDisplacementX,
                       const double &p_predictedCoMDisplacementY,
                       const double &p_predictedCoMDisplacementTheta,
                       const double &p_predictedCoMVelocity,
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
     * Predicts new robot state given
     * the previous state and a new
     * velocity.
     *
     * @param p_plannedFootstep
     * @param p_previousVelocity
     * @param p_nextVelocity
     * @param p_action
     * @param p_currentWorldCoordinatesCoM
     * @param p_currentFeetConfiguration
     * @param p_newFeetConfiguration
     * @param p_newWorldCoordinatesCoM
     */
    void predictNextState(const uint p_plannedFootstep,
                          const double p_previousVelocity,
                          const double p_nextVelocity,
                          const Action &p_action,
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

    //! Feet models' coefficients (footstep prediction)
    Eigen::RowVectorXd m_feet_x;
    Eigen::RowVectorXd m_feet_y;

    //! CoM models' coefficients (CoM prediction)
    Eigen::RowVectorXd m_com_x;
    Eigen::RowVectorXd m_com_y;
    Eigen::RowVectorXd m_com_velocity;
};