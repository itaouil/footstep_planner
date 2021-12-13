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
    m_search(p_nh),
    m_listener(m_buffer)
{
    // Robot pose subscriber and cache setup
    m_robotPoseSubscriber.subscribe(m_nh, ROBOT_POSE_TOPIC, 1);
    m_robotPoseCache.connectInput(m_robotPoseSubscriber);
    m_robotPoseCache.setCacheSize(CACHE_SIZE);

    // FL foot pose subscriber and cache setup
    m_flFootPoseSubscriber.subscribe(m_nh, FL_FOOT_POSE_TOPIC, 1);
    m_flFootPoseCache.connectInput(m_flFootPoseSubscriber);
    m_flFootPoseCache.setCacheSize(CACHE_SIZE);

    // FR foot pose subscriber and cache setup
    m_frFootPoseSubscriber.subscribe(m_nh, FR_FOOT_POSE_TOPIC, 1);
    m_frFootPoseCache.connectInput(m_frFootPoseSubscriber);
    m_frFootPoseCache.setCacheSize(CACHE_SIZE);

    // RL foot pose subscriber and cache setup
    m_rlFootPoseSubscriber.subscribe(m_nh, RL_FOOT_POSE_TOPIC, 1);
    m_rlFootPoseCache.connectInput(m_rlFootPoseSubscriber);
    m_rlFootPoseCache.setCacheSize(CACHE_SIZE);

    // RR foot pose subscriber and cache setup
    m_rrFootPoseSubscriber.subscribe(m_nh, RR_FOOT_POSE_TOPIC, 1);
    m_rrFootPoseCache.connectInput(m_rrFootPoseSubscriber);
    m_rrFootPoseCache.setCacheSize(CACHE_SIZE);
}

Planner::~Planner() = default;

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
        std::cout << ex.what() << std::endl;
        ROS_WARN("Planner: Could not transform source pose to target one. Skipping this iteration.");
        return;
    }
}

/**
 * Compute the feet placement (x,y)
 * w.r.t to the CoM frame.
 *
 * @param p_sourceFrame
 * @param p_feetConfiguration
 */
void Planner::getFeetConfiguration(FeetConfiguration &p_feetConfiguration)
{
    // Time of cache extraction
    const ros::Time l_latestPoseTime = ros::Time::now();

    // Get the latest FL foot pose from the cache
    boost::shared_ptr<wb_controller::CartesianTask const> l_latestFLFootPose =
            m_flFootPoseCache.getElemBeforeTime(l_latestPoseTime);

    // Get the latest FR foot pose from the cache
    boost::shared_ptr<wb_controller::CartesianTask const> l_latestFRFootPose =
            m_frFootPoseCache.getElemBeforeTime(l_latestPoseTime);

    // Get the latest RL foot pose from the cache
    boost::shared_ptr<wb_controller::CartesianTask const> l_latestRLFootPose =
            m_rlFootPoseCache.getElemBeforeTime(l_latestPoseTime);

    // Get the latest RR foot pose from the cache
    boost::shared_ptr<wb_controller::CartesianTask const> l_latestRRFootPose =
            m_rrFootPoseCache.getElemBeforeTime(l_latestPoseTime);

//    // Common foot pose message
//    geometry_msgs::PoseStamped l_footPoseFootFrame;
//    l_footPoseFootFrame.header.stamp = ros::Time::now();
//    l_footPoseFootFrame.pose.position.x = 0;
//    l_footPoseFootFrame.pose.position.y = 0;
//    l_footPoseFootFrame.pose.position.z = 0;
//    l_footPoseFootFrame.pose.orientation.x = 0;
//    l_footPoseFootFrame.pose.orientation.y = 0;
//    l_footPoseFootFrame.pose.orientation.z = 0;
//    l_footPoseFootFrame.pose.orientation.w = 1;
//
//    // FL configuration
//    geometry_msgs::PoseStamped l_flFootPoseMapFrame;
//    l_footPoseFootFrame.header.frame_id = FL_FOOT_FRAME;
//    getSourceToTargetPoseTransform(HEIGHT_MAP_REFERENCE_FRAME, l_footPoseFootFrame, l_flFootPoseMapFrame);
//
//    // FR configuration
//    geometry_msgs::PoseStamped l_frFootPoseMapFrame;
//    l_footPoseFootFrame.header.frame_id = FR_FOOT_FRAME;
//    getSourceToTargetPoseTransform(HEIGHT_MAP_REFERENCE_FRAME, l_footPoseFootFrame, l_frFootPoseMapFrame);
//
//    // RL configuration
//    geometry_msgs::PoseStamped l_rlFootPoseMapFrame;
//    l_footPoseFootFrame.header.frame_id = RL_FOOT_FRAME;
//    getSourceToTargetPoseTransform(HEIGHT_MAP_REFERENCE_FRAME, l_footPoseFootFrame, l_rlFootPoseMapFrame);
//
//    // RR configuration
//    geometry_msgs::PoseStamped l_rrFootPoseMapFrame;
//    l_footPoseFootFrame.header.frame_id = RR_FOOT_FRAME;
//    getSourceToTargetPoseTransform(HEIGHT_MAP_REFERENCE_FRAME, l_footPoseFootFrame, l_rrFootPoseMapFrame);

    // Populate feet CoM configuration
    p_feetConfiguration.flCoM.x = l_latestFLFootPose->pose_actual.position.x;
    p_feetConfiguration.flCoM.y = l_latestFLFootPose->pose_actual.position.y;
    p_feetConfiguration.frCoM.x = l_latestFRFootPose->pose_actual.position.x;
    p_feetConfiguration.frCoM.y = l_latestFRFootPose->pose_actual.position.y;
    p_feetConfiguration.rlCoM.x = l_latestRLFootPose->pose_actual.position.x;
    p_feetConfiguration.rlCoM.y = l_latestRLFootPose->pose_actual.position.y;
    p_feetConfiguration.rrCoM.x = l_latestRRFootPose->pose_actual.position.x;
    p_feetConfiguration.rrCoM.y = l_latestRRFootPose->pose_actual.position.y;

    // FR/RL always swing first
    p_feetConfiguration.fr_rl_swinging = true;
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
    // Get the latest robot pose from the cache
    const ros::Time l_latestPoseTime = ros::Time::now();
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> l_latestRobotPose =
            m_robotPoseCache.getElemBeforeTime(l_latestPoseTime);

    // Transform robot pose to map frame
    geometry_msgs::PoseStamped l_robotPoseMapFrame;
//    geometry_msgs::PoseStamped l_robotPoseRobotFrame;
    l_robotPoseMapFrame.header = l_latestRobotPose->header;
    l_robotPoseMapFrame.pose.position = l_latestRobotPose->pose.pose.position;
    l_robotPoseMapFrame.pose.orientation = l_latestRobotPose->pose.pose.orientation;
//    getSourceToTargetPoseTransform(HEIGHT_MAP_REFERENCE_FRAME, l_robotPoseRobotFrame, l_robotPoseMapFrame);

    // Compute grid source coordinates
    tf2::Quaternion l_startPositionQuaternion;
    tf2::convert(l_robotPoseMapFrame.pose.orientation, l_startPositionQuaternion);
    World3D l_worldStartPosition{l_robotPoseMapFrame.pose.position.x,
                                 l_robotPoseMapFrame.pose.position.y,
                                 l_robotPoseMapFrame.pose.position.z,
                                 l_startPositionQuaternion};

    // Compute grid goal coordinates
    tf2::Quaternion l_goalPositionQuaternion;
    tf2::convert(p_goalPosition.pose.orientation, l_goalPositionQuaternion);
    World3D l_worldGoalPosition{p_goalPosition.pose.position.x,
                                p_goalPosition.pose.position.y,
                                0,
                                l_goalPositionQuaternion};

    // Compute initial feet configuration
    FeetConfiguration l_feetConfiguration;
    getFeetConfiguration(l_feetConfiguration);

//    ROS_INFO_STREAM("Fl CoM Configuration: " << l_feetConfiguration.flCoM.x << ", " << l_feetConfiguration.flCoM.y);
//    ROS_INFO_STREAM("FR CoM Configuration: " << l_feetConfiguration.frCoM.x << ", " << l_feetConfiguration.frCoM.y);
//    ROS_INFO_STREAM("RL CoM Configuration: " << l_feetConfiguration.rlCoM.x << ", " << l_feetConfiguration.rlCoM.y);
//    ROS_INFO_STREAM("RR CoM Configuration: " << l_feetConfiguration.rrCoM.x << ", " << l_feetConfiguration.rrCoM.y);
//    ROS_INFO_STREAM("FR/RL swinging first: " << l_feetConfiguration.fr_rl_swinging);

//    ROS_INFO_STREAM("Fl Map Configuration: " << l_feetConfiguration.flMap.x << ", " << l_feetConfiguration.flMap.y);
//    ROS_INFO_STREAM("FR Map Configuration: " << l_feetConfiguration.frMap.x << ", " << l_feetConfiguration.frMap.y);
//    ROS_INFO_STREAM("RL Map Configuration: " << l_feetConfiguration.rlMap.x << ", " << l_feetConfiguration.rlMap.y);
//    ROS_INFO_STREAM("RR Map Configuration: " << l_feetConfiguration.rrMap.x << ", " << l_feetConfiguration.rrMap.y);

    auto start = high_resolution_clock::now();
    // Call A* search algorithm
    std::vector<Node> l_path = m_search.findPath(l_worldStartPosition,
                                                 l_worldGoalPosition,
                                                 l_feetConfiguration);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    std::cout << "Time taken by planner: "
         << duration.count() << " milliseconds" << std::endl;

    // Copy over path
    std::copy(l_path.begin(), l_path.end(), std::back_inserter(p_path));
}