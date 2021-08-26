/*
 * navigation.hpp
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
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <grid_map_msgs/GridMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// ROS services
#include <grid_map_msgs/GetGridMap.h>

// Planner
#include "planner.hpp"

// Config file
#include "config.hpp"

class Navigation
{
public:
    explicit Navigation(ros::NodeHandle&, tf2_ros::Buffer &, tf2_ros::TransformListener &);
    virtual ~Navigation();

private:
    void initialize();
    void buildInitialHeightMap();
    void planHeightMapPath(const geometry_msgs::PoseStamped&);

    // ROS node handle
    ros::NodeHandle m_nh;

    // TF variables
    tf2_ros::TransformListener &m_tf2;
    tf2_ros::Buffer &m_buffer;

    // Planner object
    Planner m_planner;

    // ROS publishers
    ros::Publisher m_pathPublisher;
    ros::Publisher m_velocityPublisher;

    // ROS subscribers
    ros::Subscriber m_goalSubscriber;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> m_robotPoseSubscriber;

    // Robot pose cache variables
    int m_robotPoseCacheSize;
    message_filters::Cache<geometry_msgs::PoseWithCovarianceStamped> m_robotPoseCache;
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> m_latestRobotPose;
};