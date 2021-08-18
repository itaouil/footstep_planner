/*
 * planner.hpp
 *
 *  Created on: Aug 16, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

// C++ general
#include <cmath>

// ROS general
#include <ros/ros.h>
#include <message_filters/cache.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS messages
#include <geometry_msgs/Twist.h>
#include <grid_map_msgs/GridMap.h>
#include "geometry_msgs/PointStamped.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Config file
#include "config.hpp"

class Planner
{
public:
    explicit Planner(ros::NodeHandle&, tf2_ros::Buffer &, tf2_ros::TransformListener &);
    virtual ~Planner();

private:
    void initialize();
    void buildInitialHeightMap();
    void planHeightMapPath(const ros::TimerEvent&);

    // ROS nodehandle
    ros::NodeHandle m_nh;

    // ROS publishers
    ros::Publisher m_velocityPublisher;

    // TF variables
    tf2_ros::TransformListener &m_tf2;
    tf2_ros::Buffer &m_buffer;

    // ROS timer that triggers planner
    ros::Timer m_plannerTimer;

    // ROS subscriber and cache for robot pose messages
    message_filters::Cache<geometry_msgs::PoseWithCovarianceStamped> m_robotPoseCache;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> m_robotPoseSubscriber;

    // Indicates whether initial height map is ready or not
    bool m_initialHeightMapBuilt;

    // Robot pose cache size
    int m_robotPoseCacheSize;
};