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
     * @return whether planning was successful
     */
    bool plan(const geometry_msgs::PointStamped&,
              const geometry_msgs::PoseStamped&);
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

    /**
     * Maps world (continuous) coordinates in
     * the map frame to the respective indices
     * (discrete) of the grid map.
     *
     * @return whether mapping was successful
     */
    bool worldToMapBound(double, double, double, double, unsigned int&, unsigned int&) const;

    //! ROS node handle
    ros::NodeHandle m_nh;

    //! ROS height map service request
    ros::ServiceClient m_heightMapServiceClient;

    //! A* search
    AStar::Search m_search;
};