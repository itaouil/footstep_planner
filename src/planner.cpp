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
    m_listener(m_buffer)
{
    // Robot pose subscriber and cache setup
    m_robotPoseSubscriber.subscribe(m_nh, ROBOT_POSE_TOPIC, 1);
    m_robotPoseCache.connectInput(m_robotPoseSubscriber);
    m_robotPoseCache.setCacheSize(CACHE_SIZE);

    // Height map service client
    m_heightMapServiceClient = m_nh.serviceClient<grid_map_msgs::GetGridMap>(HEIGHT_MAP_SERVICE_TOPIC);
}

Planner::~Planner() = default;

/**
 * Compute the feet placement (x,y)
 * w.r.t to the CoM frame.
 *
 * @param p_sourceFrame
 * @param p_feetConfiguration
 */
void Planner::getFeetConfiguration(FeetConfiguration &p_feetConfiguration)
{
    // Common foot pose message
    geometry_msgs::PoseStamped l_footPoseFootFrame;
    l_footPoseFootFrame.header.stamp = ros::Time::now();
    l_footPoseFootFrame.pose.position.x = 0;
    l_footPoseFootFrame.pose.position.y = 0;
    l_footPoseFootFrame.pose.position.z = 0;
    l_footPoseFootFrame.pose.orientation.x = 0;
    l_footPoseFootFrame.pose.orientation.y = 0;
    l_footPoseFootFrame.pose.orientation.z = 0;
    l_footPoseFootFrame.pose.orientation.w = 1;

    // FL configuration
    geometry_msgs::PoseStamped l_flFootPoseCoMFrame;
    l_footPoseFootFrame.header.frame_id = "FL_foot";
    getSourceToTargetPoseTransform(ROBOT_REFERENCE_FRAME, l_footPoseFootFrame, l_flFootPoseCoMFrame);

    // FR configuration
    geometry_msgs::PoseStamped l_frFootPoseCoMFrame;
    l_footPoseFootFrame.header.frame_id = "FR_foot";
    getSourceToTargetPoseTransform(ROBOT_REFERENCE_FRAME, l_footPoseFootFrame, l_frFootPoseCoMFrame);

    // RL configuration
    geometry_msgs::PoseStamped l_rlFootPoseCoMFrame;
    l_footPoseFootFrame.header.frame_id = "RL_foot";
    getSourceToTargetPoseTransform(ROBOT_REFERENCE_FRAME, l_footPoseFootFrame, l_rlFootPoseCoMFrame);

    // RR configuration
    geometry_msgs::PoseStamped l_rrFootPoseCoMFrame;
    l_footPoseFootFrame.header.frame_id = "RR_foot";
    getSourceToTargetPoseTransform(ROBOT_REFERENCE_FRAME, l_footPoseFootFrame, l_rrFootPoseCoMFrame);

    // Populate feet configuration structure
    p_feetConfiguration.fl.x = l_flFootPoseCoMFrame.pose.position.x;
    p_feetConfiguration.fl.y = l_flFootPoseCoMFrame.pose.position.y;
    p_feetConfiguration.fr.x = l_frFootPoseCoMFrame.pose.position.x;
    p_feetConfiguration.fr.y = l_frFootPoseCoMFrame.pose.position.y;
    p_feetConfiguration.rl.x = l_rlFootPoseCoMFrame.pose.position.x;
    p_feetConfiguration.rl.y = l_rlFootPoseCoMFrame.pose.position.y;
    p_feetConfiguration.rr.x = l_rrFootPoseCoMFrame.pose.position.x;
    p_feetConfiguration.rr.y = l_rrFootPoseCoMFrame.pose.position.y;
}

/**
 * Compute transform from source to
 * target frame.
 *
 * @param p_targetFrame
 * @param p_initialPose
 * @param p_targetPose
 */
void Planner::getSourceToTargetPoseTransform(const std::string &p_targetFrame,
                                             const geometry_msgs::PoseStamped &p_initialPose,
                                             geometry_msgs::PoseStamped &p_targetPose)
{
    try
    {
        m_buffer.transform(p_initialPose, p_targetPose, p_targetFrame, ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Planner: Could not transform source pose to target one. Skipping this iteration.");
        return;
    }
}

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
    auto t0 = high_resolution_clock::now();

    // Get the latest robot pose from the cache
    const ros::Time l_latestPoseTime = ros::Time::now();
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> l_latestRobotPose =
            m_robotPoseCache.getElemBeforeTime(l_latestPoseTime);

    auto t1 = high_resolution_clock::now();
    ROS_INFO_STREAM("Time took to fetch pose from cache: " << duration_cast<microseconds>(t1 - t0).count() << "micros");

    // Transform robot pose to map frame
    geometry_msgs::PoseStamped l_robotPoseMapFrame;
    geometry_msgs::PoseStamped l_robotPoseRobotFrame;
    l_robotPoseRobotFrame.header = l_latestRobotPose->header;
    l_robotPoseRobotFrame.pose.position = l_latestRobotPose->pose.pose.position;
    l_robotPoseRobotFrame.pose.orientation = l_latestRobotPose->pose.pose.orientation;
    getSourceToTargetPoseTransform(HEIGHT_MAP_REFERENCE_FRAME, l_robotPoseRobotFrame, l_robotPoseMapFrame);

    auto t2 = high_resolution_clock::now();
    ROS_INFO_STREAM("Time took to transform robot pose: " << duration_cast<microseconds>(t2 - t1).count() << "micros");

    ROS_DEBUG_STREAM("Robot pose in map frame: " << l_robotPoseMapFrame);

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

    auto t3 = high_resolution_clock::now();
    ROS_INFO_STREAM("Time took to request grid map: " << duration_cast<milliseconds>(t3 - t2).count() << "millis");

    // Compute grid source coordinates
    tf2::Quaternion l_startPositionQuaternion;
    tf2::convert(l_robotPoseMapFrame.pose.orientation, l_startPositionQuaternion);
    World2D l_worldStartPosition{l_robotPoseMapFrame.pose.position.x,
                                 l_robotPoseMapFrame.pose.position.y,
                                 l_startPositionQuaternion};

    auto t4 = high_resolution_clock::now();
    ROS_INFO_STREAM("Time took for start pose conversion: " << duration_cast<microseconds>(t4 - t3).count() << "micros");

    // Compute grid goal coordinates
    tf2::Quaternion l_goalPositionQuaternion;
    tf2::convert(p_goalPosition.pose.orientation, l_goalPositionQuaternion);
    World2D l_worldGoalPosition{p_goalPosition.pose.position.x,
                                p_goalPosition.pose.position.y,
                                l_goalPositionQuaternion};

    auto t5 = high_resolution_clock::now();
    ROS_INFO_STREAM("Time took for target pose conversion: " << duration_cast<microseconds>(t5 - t4).count() << "micros");

    // Compute initial feet configuration
    FeetConfiguration l_feetConfigurationCoMFrame;
    getFeetConfiguration(l_feetConfigurationCoMFrame);

    auto t6 = high_resolution_clock::now();
    ROS_INFO_STREAM("Time took to compute feet configuration: " << duration_cast<milliseconds>(t6 - t5).count() << "millis");

    ROS_DEBUG_STREAM("Fl Configuration: " << l_feetConfigurationCoMFrame.fl.x << ", " << l_feetConfigurationCoMFrame.fl.y);
    ROS_DEBUG_STREAM("FR Configuration: " << l_feetConfigurationCoMFrame.fr.x << ", " << l_feetConfigurationCoMFrame.fr.y);
    ROS_DEBUG_STREAM("RL Configuration: " << l_feetConfigurationCoMFrame.rl.x << ", " << l_feetConfigurationCoMFrame.rl.y);
    ROS_DEBUG_STREAM("RR Configuration: " << l_feetConfigurationCoMFrame.rr.x << ", " << l_feetConfigurationCoMFrame.rr.y);
    
    // Call A* search algorithm
    std::vector<Node> l_path = m_search.findPath(l_worldStartPosition,
                                                 l_worldGoalPosition,
                                                 l_feetConfigurationCoMFrame);

    auto t7 = high_resolution_clock::now();
    ROS_INFO_STREAM("Time took to plan the path: " << duration_cast<milliseconds>(t7 - t6).count() << "millis");

    // Copy over path
    std::copy(l_path.begin(), l_path.end(), std::back_inserter(p_path));
}


