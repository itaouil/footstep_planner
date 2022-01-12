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
#include <wb_controller/CartesianTask.h>
#include <visualization_msgs/MarkerArray.h>

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
     * Plans path from start to goal.
     *
     * @param p_goalPosition
     * @param p_initialAction
     * @param p_initialVelocity
     * @param p_swingingPair
     * @param p_path
     */
    void plan(const geometry_msgs::PoseStamped &p_goalPosition,
              const Action &p_initialAction,
              const double &p_initialVelocity,
              const bool &p_swingingPair,
              std::vector<Node> &p_path);
private:
    /**
     * Compute transform from source to
     * target frame.
     *
     * @param p_targetFrame
     * @param p_initialPose
     * @param p_targetPose
     */
    void getSourceToTargetPoseTransform(const std::string &p_targetFrame,
                                        const geometry_msgs::PoseStamped &p_initialPose,
                                        geometry_msgs::PoseStamped &p_targetPose);

    /**
     * Compute feet placement (x,y)
     * w.r.t to the CoM frame.
     *
     * @param p_feetConfiguration
     * @param p_swingingFRRL
     */
    void getFeetConfiguration(FeetConfiguration &p_feetConfiguration, const bool &p_swingingFRRL);

    //! ROS node handle
    ros::NodeHandle m_nh;

    //! A* search
    AStar::Search m_search;

    //! TF2 buffer
    tf2_ros::Buffer m_buffer;

    //! TF2 listener
    tf2_ros::TransformListener m_listener;

    //! Robot pose cache
    message_filters::Cache<nav_msgs::Odometry> m_robotPoseCache;
    message_filters::Subscriber<nav_msgs::Odometry> m_robotPoseSubscriber;

    //! FL foot pose cache
    message_filters::Cache<wb_controller::CartesianTask> m_flFootPoseCache;
    message_filters::Subscriber<wb_controller::CartesianTask> m_flFootPoseSubscriber;

    //! FR foot pose cache
    message_filters::Cache<wb_controller::CartesianTask> m_frFootPoseCache;
    message_filters::Subscriber<wb_controller::CartesianTask> m_frFootPoseSubscriber;

    //! RL foot pose cache
    message_filters::Cache<wb_controller::CartesianTask> m_rlFootPoseCache;
    message_filters::Subscriber<wb_controller::CartesianTask> m_rlFootPoseSubscriber;

    //! RR foot pose cache
    message_filters::Cache<wb_controller::CartesianTask> m_rrFootPoseCache;
    message_filters::Subscriber<wb_controller::CartesianTask> m_rrFootPoseSubscriber;
};