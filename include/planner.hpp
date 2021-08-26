/*
 * planner.hpp
 *
 *  Created on: Aug 23, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

// ROS general
#include <ros/ros.h>

// ROS messages
#include <grid_map_msgs/GridMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

// ROS services
#include <grid_map_msgs/GetGridMap.h>

// A*
#include <search/AStar.hpp>

// Structs
#include <structs/vec2D.hpp>
#include <structs/world2D.hpp>

// Config
#include "config.hpp"

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
    void plan(const geometry_msgs::PointStamped &p_robotPose,
              const geometry_msgs::PoseStamped &p_goalPosition,
              std::vector<World2D> &p_path);
private:
    /**
     * Requests height elevation map from
     * the elevation map package and populates
     * a reference with the obtained grid map.
     *
     * @param p_heightMap
     * @return if height map request was successful
     */
    bool getHeightMap(grid_map_msgs::GridMap&, const geometry_msgs::PointStamped&);

    //! ROS node handle
    ros::NodeHandle m_nh;

    //! ROS height map service request
    ros::ServiceClient m_heightMapServiceClient;

    //! A* search
    AStar::Search m_search;
};