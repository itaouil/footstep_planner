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
        ROS_INFO("Planner: Height map requested succeeded.");
    }

    // Compute grid starting place
    Vec2D l_gridStartPosition{};
    World2D l_worldStartPosition{p_robotPose.point.x, p_robotPose.point.y};
    AStar::worldToGrid(
                       l_heightMap.info.pose.position.x,
                       l_heightMap.info.pose.position.y,
                       l_worldStartPosition,
                       l_gridStartPosition);

    // Compute grid goal position
    Vec2D l_gridGoalPosition{};
    World2D l_worldGoalPosition{p_goalPosition.pose.position.x, p_goalPosition.pose.position.y};
    AStar::worldToGrid(l_heightMap.info.pose.position.x,
                       l_heightMap.info.pose.position.y,
                       l_worldGoalPosition,
                       l_gridGoalPosition);

    ROS_INFO_STREAM("A* start search called with given source: " << l_gridStartPosition.x << ", " << l_gridStartPosition.y);
    ROS_INFO_STREAM("A* start search called with given target: " << l_gridGoalPosition.x << ", " << l_gridGoalPosition.y);

    // Set grid origin for the search conversion
    m_search.setGridOrigin(l_heightMap.info.pose.position.x, l_heightMap.info.pose.position.y);

    // Call A* search algorithm
    m_search.findPath(l_gridStartPosition, l_gridGoalPosition);

    // Call search algorithm and pass
    // it the start and goal indexes
    //TODO: call preferred search algorithm

    return false;
}


