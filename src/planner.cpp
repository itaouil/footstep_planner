/*
 * planner.cpp
 *
 *  Created on: Aug 23, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#include "planner.hpp"

Planner::Planner() = default;

Planner::~Planner() = default;

/**
 * Maps world (continuous) coordinates in
 * the map frame to the respective discrete
 * (indices) of the grid.
 *
 * @param wx
 * @param wy
 * @param mx
 * @param my
 * @return whether mapping was successful
 */
bool Planner::worldToMapBound(const double p_wx,
                              const double p_wy,
                              const double p_originX,
                              const double p_originY,
                              unsigned int& p_mx,
                              unsigned int& p_my) const {
    // Make sure that map origin is not
    // bigger than the world coordinates
    // to avoid negative grid indices
    if (p_wx < p_originX || p_wy < p_originY)
    {
        ROS_ERROR("Planner: Could not convert world pose to grid indices.");
        return false;
    }

    // Convert from world coordinates to grid indices
    p_mx = (int)((p_wx - p_originX) / HEIGHT_MAP_RESOLUTION);
    p_my = (int)((p_wy - p_originY) / HEIGHT_MAP_RESOLUTION);

    // Check that indices are not bigger
    // than the actual grid max size in x and y
    if (p_mx < HEIGHT_MAP_MAX_SIZE_X && p_my < HEIGHT_MAP_MAX_SIZE_Y)
        return true;
    else
    {
        ROS_ERROR("Planner: Mapped grid indices are bigger than grid max size.");
        return false;
    }
}

/**
 * Plans path from start to goal.
 *
 * @param p_heightMap
 * @param p_robotPose
 * @param p_goalPosition
 * @return whether planning was successful
 */
bool Planner::plan(const grid_map_msgs::GridMap &p_heightMap,
                   const geometry_msgs::PointStamped &p_robotPose,
                   const geometry_msgs::PoseStamped &p_goalPosition) {
    // Compute grid indexes for current robot pose
    unsigned int l_mx = 0;
    unsigned int l_my = 0;
    worldToMapBound(p_robotPose.point.x,
                    p_robotPose.point.y,
                    p_heightMap.info.pose.position.x,
                    p_heightMap.info.pose.position.y,
                    l_mx,
                    l_my);

    // Call search algorithm and pass
    // it the start and goal indexes
    //TODO: call preferred search algorithm

    return false;
}


