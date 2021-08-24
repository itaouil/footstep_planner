/*
 * planner.cpp
 *
 *  Created on: Aug 23, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#include "planner.hpp"

/**
 * Constructor
 *
 * @param p_nh
 */
Planner::Planner(ros::NodeHandle& p_nh): m_nh(p_nh)
{
    // Height map service client
    m_heightMapServiceClient = m_nh.serviceClient<grid_map_msgs::GetGridMap>("/elevation_mapping/get_raw_submap");
}

Planner::~Planner() = default;

/**
 * Requests height elevation map from
 * the elevation map package and populates
 * a reference with the obtained grid map.
 *
 * @param p_heightMap
 * @return if height map request was successful
 */
bool Planner::getHeightMap(grid_map_msgs::GridMap &p_heightMap,
                           const geometry_msgs::PointStamped &p_robotPoseMapFrame)
{
    // Service message passed to height map service
    grid_map_msgs::GetGridMap l_heightMapSrv;
    l_heightMapSrv.request.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    l_heightMapSrv.request.position_x = p_robotPoseMapFrame.point.x;
    l_heightMapSrv.request.position_y = p_robotPoseMapFrame.point.y;
    l_heightMapSrv.request.length_x = HEIGHT_MAP_LENGTH_X;
    l_heightMapSrv.request.length_y = HEIGHT_MAP_LENGTH_Y;
    l_heightMapSrv.request.layers.push_back(std::string("elevation"));

    // Call height map service request
    if (m_heightMapServiceClient.call(l_heightMapSrv))
    {
        p_heightMap = l_heightMapSrv.response.map;
        return true;
    }
    else
    {
        return false;
    }
}

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
bool Planner::plan(const geometry_msgs::PointStamped &p_robotPose,
                   const geometry_msgs::PoseStamped &p_goalPosition)
{
    // Request height map
    grid_map_msgs::GridMap l_heightMap;
    if (!getHeightMap(l_heightMap, p_robotPose))
    {
        ROS_WARN("Planner: Height map request failed. Skipping iteration");
        return false;
    }
    else
    {
        ROS_INFO("Planner: Height map requestes succeeded.");
    }

    // Compute grid starting place
    unsigned int l_startPositionX = 0;
    unsigned int l_startPositionY = 0;
    worldToMapBound(p_robotPose.point.x,
                    p_robotPose.point.y,
                    l_heightMap.info.pose.position.x,
                    l_heightMap.info.pose.position.y,
                    l_startPositionX,
                    l_startPositionY);

    // Compute grid goal position
    unsigned int l_goalPositionX = 0;
    unsigned int l_goalPositionY = 0;
    worldToMapBound(p_goalPosition.pose.position.x,
                    p_goalPosition.pose.position.y,
                    l_heightMap.info.pose.position.x,
                    l_heightMap.info.pose.position.y,
                    l_goalPositionX,
                    l_goalPositionY);

    //TODO: Call A* start search algorithm to find path

    // Call search algorithm and pass
    // it the start and goal indexes
    //TODO: call preferred search algorithm

    return false;
}


