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
void Planner::getFeetConfiguration(FeetConfiguration &p_feetConfiguration, const bool &p_swingingFRRL)
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
    p_feetConfiguration.fr_rl_swinging = p_swingingFRRL;
}

/**
 * Plans path from start to goal.
 *
 * @param p_goalPosition
 * @param p_currentNode
 * @param p_path
 */
void Planner::plan(const geometry_msgs::PoseStamped &p_goalPosition,
                   const Action &p_initialAction,
                   const double &p_initialVelocity,
                   const bool &p_swingingFRRL,
                   std::vector<Node> &p_path)
{
    // Get the latest robot pose from the cache
    const ros::Time l_latestPoseTime = ros::Time::now();
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> l_latestRobotPose =
            m_robotPoseCache.getElemBeforeTime(l_latestPoseTime);

    // Transform robot pose to map frame
    geometry_msgs::PoseStamped l_robotPoseMapFrame;
    l_robotPoseMapFrame.header = l_latestRobotPose->header;
    l_robotPoseMapFrame.pose.position = l_latestRobotPose->pose.pose.position;
    l_robotPoseMapFrame.pose.orientation = l_latestRobotPose->pose.pose.orientation;

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

    // Compute feet configuration
    FeetConfiguration l_feetConfiguration;
    getFeetConfiguration(l_feetConfiguration, p_swingingFRRL);

    // Call A* search algorithm
    auto start = high_resolution_clock::now();
    std::vector<Node> l_path = m_search.findPath(p_initialAction,
                                                 p_initialVelocity,
                                                 l_worldStartPosition,
                                                 l_worldGoalPosition,
                                                 l_feetConfiguration);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    ROS_INFO_STREAM("Time taken by the planner: " << duration.count() << " milliseconds" << std::endl);

    // Copy over path
    std::copy(l_path.begin(), l_path.end(), std::back_inserter(p_path));
}