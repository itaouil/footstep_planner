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

// Config
#include "config.hpp"

class Planner
{
public:
    /**
     * Constructor.
     */
    explicit Planner();

    /**
     * Destructor.
     */
    virtual ~Planner();

    /**
     * Plans path from start to goal.
     *
     * @return whether planning was successful
     */
    bool plan(const grid_map_msgs::GridMap&,
              const geometry_msgs::PointStamped&,
              const geometry_msgs::PoseStamped&);
private:
    /**
     * Maps world (continuous) coordinates in
     * the map frame to the respective indices
     * (discrete) of the grid map.
     *
     * @return whether mapping was successful
     */
    bool worldToMapBound(double, double, double, double, unsigned int&, unsigned int&) const;
};