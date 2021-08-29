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
Planner::Planner(ros::NodeHandle& p_nh):
    m_nh(p_nh),
    m_listener(m_buffer),
    m_robotPoseCacheSize(ROBOT_POSE_CACHE_SIZE)
{
    // Robot pose subscriber and cache setup
    m_robotPoseSubscriber.subscribe(m_nh, ROBOT_POSE_TOPIC, 1);
    m_robotPoseCache.connectInput(m_robotPoseSubscriber);
    m_robotPoseCache.setCacheSize(m_robotPoseCacheSize);

    // Height map service client
    m_heightMapServiceClient = m_nh.serviceClient<grid_map_msgs::GetGridMap>(HEIGHT_MAP_SERVICE_TOPIC);
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
bool Planner::getHeightMap(grid_map_msgs::GridMap &p_heightMap, const geometry_msgs::PoseStamped &p_robotPoseMapFrame)
{
    // Service message passed to height map service
    grid_map_msgs::GetGridMap l_heightMapSrv;
    l_heightMapSrv.request.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    l_heightMapSrv.request.position_x = p_robotPoseMapFrame.pose.position.x;
    l_heightMapSrv.request.position_y = p_robotPoseMapFrame.pose.position.y;
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
 * @param std
 * @param p_path
 */
void Planner::plan(const geometry_msgs::PoseStamped &p_goalPosition,
                   std::vector<World2D> &p_path)
{
    // Compute robot to map transform
    geometry_msgs::TransformStamped l_transformStamped;
    try
    {
        // Compute transform
        l_transformStamped = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME, ROBOT_REFERENCE_FRAME, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Planner: Could not compute robot to map frame transform.");
    }

    // Apply transform previously computed
    // to obtain robot pose in map frame
    geometry_msgs::PoseStamped l_robotPoseMapFrame;
    try
    {
        // Transform pose from robot frame to map frame
        geometry_msgs::PoseStamped l_poseRobotFrame;
        l_poseRobotFrame.header.stamp = ros::Time::now();
        l_poseRobotFrame.header.frame_id = ROBOT_REFERENCE_FRAME;
        l_poseRobotFrame.pose.position.x = 0;
        l_poseRobotFrame.pose.position.y = 0;
        l_poseRobotFrame.pose.position.z = 0;
        l_poseRobotFrame.pose.orientation.x = 0;
        l_poseRobotFrame.pose.orientation.y = 0;
        l_poseRobotFrame.pose.orientation.z = 0;
        l_poseRobotFrame.pose.orientation.w = 1;
        tf2::doTransform(l_poseRobotFrame, l_robotPoseMapFrame, l_transformStamped);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Planner: Could not apply transform to get robot pose in map frame.");
    }

    // Request height map
    grid_map_msgs::GridMap l_heightMap;
    if (!getHeightMap(l_heightMap, l_robotPoseMapFrame))
    {
        ROS_WARN("Planner: Height map request failed. Skipping planning request");
        return;
    }
    else
    {
        ROS_INFO("Planner: Height map request succeeded.");
    }

    ROS_INFO_STREAM("Rotation transform: " << l_transformStamped.transform.rotation);

    // Compute grid starting place
    World2D l_worldStartPosition{l_robotPoseMapFrame.pose.position.x,
                                 l_robotPoseMapFrame.pose.position.y,
                                 Quaternion{l_transformStamped.transform.rotation.x,
                                               l_transformStamped.transform.rotation.y,
                                               l_transformStamped.transform.rotation.z,
                                               l_transformStamped.transform.rotation.w}};

    // Compute grid goal position
    World2D l_worldGoalPosition{p_goalPosition.pose.position.x,
                                p_goalPosition.pose.position.y,
                                Quaternion{0, 0, 0, 1}};

    // Set grid origin for the search conversion
    m_search.setGridOrigin(l_heightMap.info.pose.position.x, l_heightMap.info.pose.position.y);

    // Call A* search algorithm
    std::vector<World2D> l_path = m_search.findPath(l_worldStartPosition, l_worldGoalPosition);

    // Copy over path
    std::copy(l_path.begin(), l_path.end(), std::back_inserter(p_path));
}