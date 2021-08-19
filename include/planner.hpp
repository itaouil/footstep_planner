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

// Grid map
#include <grid_map_msgs/GetGridMap.h>

// Config file
#include "config.hpp"

class Planner
{
public:
    explicit Planner(ros::NodeHandle&, tf2_ros::Buffer &, tf2_ros::TransformListener &);
    virtual ~Planner();

private:
    void initialize();
    void buildInitialHeightMap(const ros::TimerEvent&);
    void planHeightMapPath(const ros::TimerEvent&);

    // ROS nodehandle
    ros::NodeHandle m_nh;

    // TF variables
    tf2_ros::TransformListener &m_tf2;
    tf2_ros::Buffer &m_buffer;

    // ROS publisher for velocity commands
    ros::Publisher m_velocityPublisher;

    // ROS publisher for requested height map
    ros::Publisher m_heightMapPublisher;

    // ROS subscriber and cache for robot pose messages
    message_filters::Cache<geometry_msgs::PoseWithCovarianceStamped> m_robotPoseCache;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> m_robotPoseSubscriber;

    // ROS service client that requests height map
    ros::ServiceClient m_heightMapServiceClient;

    // ROS timer that triggers planner
    ros::Timer m_plannerTimer;

    // ROS timer that triggers initial height map building
    ros::Timer m_initialHeightMapTimer;

    // Robot pose cache size
    int m_robotPoseCacheSize;

    // Robot pose from cache
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> m_latestRobotPose;
};