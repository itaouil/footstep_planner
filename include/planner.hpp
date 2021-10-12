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
#include <grid_map_msgs/GridMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

// ROS services
#include <grid_map_msgs/GetGridMap.h>

// A*
#include <search/AStar.hpp>

// Structs
#include <structs/node.hpp>
#include <structs/vec2D.hpp>
#include <structs/world2D.hpp>
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
     * @param std
     * @param p_path
     */
    void plan(const geometry_msgs::PoseStamped &p_goalPosition, std::vector<Node> &p_path);
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
     */
    void getFeetConfiguration(FeetConfiguration &p_feetConfiguration);

    /**
     * Requests height elevation map from
     * the elevation map package and populates
     * a reference with the obtained grid map.
     *
     * @param p_heightMap
     * @return if height map request was successful
     */
    bool getHeightMap(grid_map_msgs::GridMap &p_heightMap, const geometry_msgs::PoseStamped &p_robotPoseMapFrame);

    //! ROS node handle
    ros::NodeHandle m_nh;

    //! A* search
    AStar::Search m_search;

    //! TF2 buffer
    tf2_ros::Buffer m_buffer;

    //! TF2 listener
    tf2_ros::TransformListener m_listener;

    //! ROS height map service request
    ros::ServiceClient m_heightMapServiceClient;

    //! Robot pose cache
    message_filters::Cache<geometry_msgs::PoseWithCovarianceStamped> m_robotPoseCache;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> m_robotPoseSubscriber;
};