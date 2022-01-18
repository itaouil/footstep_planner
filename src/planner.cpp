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
Planner::Planner(ros::NodeHandle &p_nh) :
        m_nh(p_nh),
        m_search(p_nh),
        m_listener(m_buffer) {
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
 * Compute the feet placement (x,y)
 * w.r.t to the CoM frame.
 *
 * @param p_sourceFrame
 * @param p_feetConfiguration
 */
void Planner::getFeetConfiguration(boost::shared_ptr<nav_msgs::Odometry const> &p_robotPose,
                                   FeetConfiguration &p_feetConfiguration,
                                   const bool &p_swingingFRRL) {
    // Time of cache extraction
    const ros::Time l_latestPoseTime = ros::Time::now();

    // Get the latest FL foot pose from the cache
    boost::shared_ptr<wb_controller::CartesianTask const> l_flFootPose =
            m_flFootPoseCache.getElemBeforeTime(
                    l_latestPoseTime);

    // Get the latest FR foot pose from the cache
    boost::shared_ptr<wb_controller::CartesianTask const> l_frFootPose =
            m_frFootPoseCache.getElemBeforeTime(
                    l_latestPoseTime);

    // Get the latest RL foot pose from the cache
    boost::shared_ptr<wb_controller::CartesianTask const> l_rlFootPose =
            m_rlFootPoseCache.getElemBeforeTime(
                    l_latestPoseTime);

    // Get the latest RR foot pose from the cache
    boost::shared_ptr<wb_controller::CartesianTask const> l_rrFootPose =
            m_rrFootPoseCache.getElemBeforeTime(
                    l_latestPoseTime);

    // Populate map feet poses entry
    p_feetConfiguration.flMap.x = p_robotPose->pose.pose.position.x + l_flFootPose->pose_actual.position.x;
    p_feetConfiguration.flMap.y = p_robotPose->pose.pose.position.y + l_flFootPose->pose_actual.position.y;
    p_feetConfiguration.frMap.x = p_robotPose->pose.pose.position.x + l_frFootPose->pose_actual.position.x;
    p_feetConfiguration.frMap.y = p_robotPose->pose.pose.position.y + l_frFootPose->pose_actual.position.y;
    p_feetConfiguration.rlMap.x = p_robotPose->pose.pose.position.x + l_rlFootPose->pose_actual.position.x;
    p_feetConfiguration.rlMap.y = p_robotPose->pose.pose.position.y + l_rlFootPose->pose_actual.position.y;
    p_feetConfiguration.rrMap.x = p_robotPose->pose.pose.position.x + l_rrFootPose->pose_actual.position.x;
    p_feetConfiguration.rrMap.y = p_robotPose->pose.pose.position.y + l_rrFootPose->pose_actual.position.y;

    // Populate CoM feet poses entry
    p_feetConfiguration.flCoM.x = l_flFootPose->pose_actual.position.x;
    p_feetConfiguration.flCoM.y = l_flFootPose->pose_actual.position.y;
    p_feetConfiguration.frCoM.x = l_frFootPose->pose_actual.position.x;
    p_feetConfiguration.frCoM.y = l_frFootPose->pose_actual.position.y;
    p_feetConfiguration.rlCoM.x = l_rlFootPose->pose_actual.position.x;
    p_feetConfiguration.rlCoM.y = l_rlFootPose->pose_actual.position.y;
    p_feetConfiguration.rrCoM.x = l_rrFootPose->pose_actual.position.x;
    p_feetConfiguration.rrCoM.y = l_rrFootPose->pose_actual.position.y;

//    ROS_INFO_STREAM("Planner: CoM (MAP) " << p_robotPose->pose.pose.position.x << ", " << p_robotPose->pose.pose.position.y);
//
//    ROS_INFO_STREAM("Planner: FL (MAP) " << p_feetConfiguration.flMap.x << ", " << p_feetConfiguration.flMap.y);
//    ROS_INFO_STREAM("Planner: FL (COM) " << p_feetConfiguration.flCoM.x << ", " << p_feetConfiguration.flCoM.y << "\n");
//
//    ROS_INFO_STREAM("Planner: FR (MAP) " << p_feetConfiguration.frMap.x << ", " << p_feetConfiguration.frMap.y);
//    ROS_INFO_STREAM("Planner: FL (COM) " << p_feetConfiguration.frCoM.x << ", " << p_feetConfiguration.frCoM.y << "\n");
//
//    ROS_INFO_STREAM("Planner: RL (MAP) " << p_feetConfiguration.rlMap.x << ", " << p_feetConfiguration.rlMap.y);
//    ROS_INFO_STREAM("Planner: RL (COM) " << p_feetConfiguration.rlCoM.x << ", " << p_feetConfiguration.rlCoM.y << "\n");
//
//    ROS_INFO_STREAM("Planner: RR (MAP) " << p_feetConfiguration.rrMap.x << ", " << p_feetConfiguration.rrMap.y);
//    ROS_INFO_STREAM("Planner: RR (COM) " << p_feetConfiguration.rrCoM.x << ", " << p_feetConfiguration.rrCoM.y << "\n");

    // FR/RL always swing first
    if (l_frFootPose->pose_actual.position.z > l_flFootPose->pose_actual.position.z ||
        l_frFootPose->pose_actual.position.x > l_flFootPose->pose_actual.position.z) {
        p_feetConfiguration.fr_rl_swinging = true;
    }
}

/**
 * Plans path from start to goal.
 *
 * @param p_goalPosition
 * @param p_initialAction
 * @param p_initialVelocity
 * @param p_swingingPair
 * @param p_path
 */
void Planner::plan(const geometry_msgs::PoseStamped &p_goalPosition,
                   const Action &p_initialAction,
                   const double &p_initialVelocity,
                   const bool &p_swingingFRRL,
                   std::vector<Node> &p_path) {
    auto start = high_resolution_clock::now();
    // Current odometry info
    boost::shared_ptr<nav_msgs::Odometry const> l_robotPose = m_robotPoseCache.getElemBeforeTime(
            ros::Time::now());

    // Compute grid source coordinates
    tf2::Quaternion l_startPositionQuaternion;
    tf2::convert(l_robotPose->pose.pose.orientation, l_startPositionQuaternion);
    World3D l_worldStartPosition{l_robotPose->pose.pose.position.x,
                                 l_robotPose->pose.pose.position.y,
                                 l_robotPose->pose.pose.position.z,
                                 l_startPositionQuaternion};

    // Compute grid goal coordinates
    tf2::Quaternion l_goalPositionQuaternion;
    tf2::convert(p_goalPosition.pose.orientation, l_goalPositionQuaternion);
    World3D l_worldGoalPosition{p_goalPosition.pose.position.x,
                                p_goalPosition.pose.position.y,
                                0,
                                l_goalPositionQuaternion};

    ROS_INFO_STREAM("Received goal pose: " << p_goalPosition.pose.position.x << ", "
                                           << p_goalPosition.pose.position.y << ", "
                                           << p_goalPosition.pose.position.z);
    ROS_INFO_STREAM("Current robot pose: " << l_robotPose->pose.pose.position.x << ", "
                                           << l_robotPose->pose.pose.position.y);

    // Compute feet configuration
    FeetConfiguration l_feetConfiguration;
    getFeetConfiguration(l_robotPose, l_feetConfiguration, p_swingingFRRL);

    // Clear passed path before calling search
    p_path.clear();

    // Call the A* search algorithm
    m_search.findPath(p_initialAction,
                      p_initialVelocity,
                      l_worldStartPosition,
                      l_worldGoalPosition,
                      l_robotPose->twist.twist,
                      l_feetConfiguration,
                      p_path);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    ROS_INFO_STREAM("Time taken by the planner: " << duration.count() << " microseconds" << std::endl);
}