/*
 * planner.hpp
 *
 *  Created on: Aug 23, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

// C++
#include <queue>
#include <numeric>

// ROS general
#include <chrono>
#include <ros/ros.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS messages
#include <nav_msgs/Odometry.h>
#include <grid_map_msgs/GridMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <unitree_legged_msgs/Cartesian.h>
#include <visualization_msgs/MarkerArray.h>
#include <unitree_legged_msgs/HighStateStamped.h>

// ROS services
#include <grid_map_msgs/GetGridMap.h>

// A*
#include <aStar.hpp>

// Structs
#include <structs/node.hpp>
#include <structs/vec2D.hpp>
#include <structs/world3D.hpp>
#include <structs/quaternion.hpp>

// Config
#include "config.hpp"

using namespace std::chrono;

class Planner
{
public:
    /**
     * Constructor.
     */
    explicit Planner(ros::NodeHandle&);

    /**
     * Destructor.
     */
    virtual ~Planner();

    /**
     * Prints highest, lowest and
     * average planner runtime.
     */
    void stats();

    /**
     * Plans path from start to goal.
     *
     * @param p_path
     * @param p_swingingPair
     * @param p_initialAction
     * @param p_initialVelocity
     * @param p_robotPose
     * @param p_goalPosition
     * @param p_latestCoMFeetPoses
     */
    void plan(std::vector<Node> &p_path,
              const bool &p_swingingPair,
              const Action &p_initialAction,
              const double &p_initialVelocity,
              const nav_msgs::Odometry &p_robotPose,
              const geometry_msgs::PoseStamped &p_goalPosition,
              const std::vector<unitree_legged_msgs::Cartesian> &p_latestCoMFeetPoses);
private:
    /**
     * Populate feet configuration struct
     *
     * @param p_swingingFRRL
     * @param p_robotPose
     * @param p_feetConfiguration
     * @param p_latestCoMFeetPoses
     */
    void getFeetConfiguration(const bool &p_swingingFRRL,
                              const nav_msgs::Odometry &p_robotPose,
                              FeetConfiguration &p_feetConfiguration,
                              const std::vector<unitree_legged_msgs::Cartesian> &p_latestCoMFeetPoses);

    /**
     * ROS variables
     */

    //! ROS node handle
    ros::NodeHandle m_nh;

    //! TF2 buffer
    tf2_ros::Buffer m_buffer;

    //! TF2 listener
    tf2_ros::TransformListener m_listener;

    /**
     * A* variables
     */

    //! A* search
    AStar::Search m_search;

    //! Planner runtimes
    std::vector<unsigned int> m_runtimes;

    //! Robot pose cache
    message_filters::Cache<nav_msgs::Odometry> m_robotPoseCache;
    message_filters::Subscriber<nav_msgs::Odometry> m_robotPoseSubscriber;
};