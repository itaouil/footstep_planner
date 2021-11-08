/*
 * navigation.hpp
 *
 *  Created on: Aug 16, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

// C++ general
#include <queue>
#include <cmath>

// ROS general
#include <ros/ros.h>
#include <message_filters/cache.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS messages
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Planner
#include "planner.hpp"

// Elevation map processor
#include "elevationMapProcessor.hpp"

// Config file
#include "config.hpp"

class Navigation
{
public:
    explicit Navigation(ros::NodeHandle&, tf2_ros::Buffer &, tf2_ros::TransformListener &);
    virtual ~Navigation();

private:
    /**
     * Initial publisher, subscriber,
     * and services initialization.
     */
    void initialize();

    /**
     * Perform rotation movement to
     * build initial height map if
     * required.
     */
    void buildInitialHeightMap();

    /**
     * Plans a path to a target goal
     * using an elevation map (2.5D).
     *
     * @param p_goalMsg
     */
    void planHeightMapPath(const geometry_msgs::PoseStamped &p_goalMsg);

    /**
     * Publish predicted CoM and
     * footstep sequence.
     *
     * @param p_path
     */
    void publishPredictedFootstepSequence(const std::vector<Node> &p_path);

    //! ROS node handle
    ros::NodeHandle m_nh;

    //! ROS rate
    ros::Rate m_rate;

    //! TF variables
    tf2_ros::TransformListener &m_tf2;
    tf2_ros::Buffer &m_buffer;

    //! Planner object
    Planner m_planner;

    //! Elevation map processor object
    ElevationMapProcessor m_elevationMapProcessor;

    //! ROS subscribers
    ros::Subscriber m_goalSubscriber;

    //! ROS publishers
    ros::Publisher m_velocityPublisher;
    ros::Publisher m_realPathPublisher;
    ros::Publisher m_targetPathPublisher;
    ros::Publisher m_realFeetConfigurationPublisher;
    ros::Publisher m_targetFeetConfigurationPublisher;

    //! Odometry cache
    message_filters::Cache<nav_msgs::Odometry> m_odomCache;
    message_filters::Subscriber<nav_msgs::Odometry> m_odomSubscriber;

    //! Planned motion commands
    std::queue<geometry_msgs::Twist> m_motionCommands;

    //! Transformed CoM
    std::vector<geometry_msgs::PointStamped> m_transCoM;

    //! Feet transformed
    std::vector<visualization_msgs::MarkerArray> m_targetFootsteps;
};