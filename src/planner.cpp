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
                   std::vector<Node> &p_path)
{
    // Get latest robot pose from the cache
    const ros::Time l_latestPoseTime = ros::Time::now();
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> l_latestRobotPose =
            m_robotPoseCache.getElemBeforeTime(l_latestPoseTime);

    // Transform robot pose to map frame
    geometry_msgs::PoseStamped l_robotPoseMapFrame;
    try
    {
        geometry_msgs::PoseStamped l_robotPoseRobotFrame;
        l_robotPoseRobotFrame.header = l_latestRobotPose->header;
        l_robotPoseRobotFrame.pose.position = l_latestRobotPose->pose.pose.position;
        l_robotPoseRobotFrame.pose.orientation = l_latestRobotPose->pose.pose.orientation;
        m_buffer.transform(l_robotPoseRobotFrame, l_robotPoseMapFrame, HEIGHT_MAP_REFERENCE_FRAME, ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Planner: Could not transform robot pose to map frame. Skipping this iteration.");
        return;
    }

    ROS_INFO_STREAM("Transform: " << l_robotPoseMapFrame);

    // Request height map
    grid_map_msgs::GridMap l_heightMap;
    if (!getHeightMap(l_heightMap, l_robotPoseMapFrame))
    {
        ROS_WARN("Planner: Height map request failed. Skipping planning request");
        return;
    }
    else
    {
        // Set grid origin for the search conversion
        m_search.setGridOrigin(l_heightMap.info.pose.position.x, l_heightMap.info.pose.position.y);

        //TODO: Set remaining search parameters (grid size, etc)

        ROS_INFO("Planner: Height map request succeeded. Initial search parameters set.");
    }

    // Compute grid starting place
    tf2::Quaternion l_startPositionQuaternion;
    tf2::convert(l_robotPoseMapFrame.pose.orientation, l_startPositionQuaternion);
    World2D l_worldStartPosition{l_robotPoseMapFrame.pose.position.x,
                                 l_robotPoseMapFrame.pose.position.y,
                                 l_startPositionQuaternion};

    // Compute grid goal position
    tf2::Quaternion l_goalPositionQuaternion;
    tf2::convert(p_goalPosition.pose.orientation, l_goalPositionQuaternion);
    World2D l_worldGoalPosition{p_goalPosition.pose.position.x,
                                p_goalPosition.pose.position.y,
                                l_goalPositionQuaternion};

    // Call A* search algorithm
    std::vector<Node> l_path = m_search.findPath(l_worldStartPosition, l_worldGoalPosition);

    // Copy over path
    std::copy(l_path.begin(), l_path.end(), std::back_inserter(p_path));
}