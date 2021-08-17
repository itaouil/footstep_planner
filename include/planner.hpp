/*
 * planner.hpp
 *
 *  Created on: Aug 16, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#ifndef ALIENGO_WS_PLANNER_H
#define ALIENGO_WS_PLANNER_H

// C++ general
#include <cmath>

// ROS general
#include <ros/ros.h>

// ROS messages
#include <geometry_msgs/Twist.h>
#include <grid_map_msgs/GridMap.h>

// Config file
#include "config.hpp"

class Planner
{
public:
    explicit Planner(ros::NodeHandle&);
    virtual ~Planner();

    void elevation_map_callback(const grid_map_msgs::GridMap::ConstPtr&);

private:
    void initialize();
    void buildInitialHeightMap();

    // ROS
    ros::NodeHandle m_nh;
    ros::Publisher m_vel_pub;
};

#endif //ALIENGO_WS_PLANNER_H
