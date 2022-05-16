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

    // High state subscriber and cache setup
    m_highStateSubscriber.subscribe(m_nh, HIGH_STATE_TOPIC, 1);
    m_highStateCache.connectInput(m_highStateSubscriber);
    m_highStateCache.setCacheSize(CACHE_SIZE);
}

Planner::~Planner() = default;

/**
 * Compute the feet placement (x,y)
 * w.r.t to the CoM frame.
 *
 * @param p_sourceFrame
 * @param p_feetConfiguration
 */
void Planner::getFeetConfiguration(const nav_msgs::Odometry &p_robotPose,
                                   FeetConfiguration &p_feetConfiguration,
                                   const bool &p_swingingFRRL) {
    // Get the latest high state from the cache
    boost::shared_ptr<unitree_legged_msgs::HighStateStamped const> l_highState =
            m_highStateCache.getElemBeforeTime(ros::Time::now());

    // Transform feet poses from CoM frame to map frame
    geometry_msgs::TransformStamped l_flMap;
    geometry_msgs::TransformStamped l_frMap;
    geometry_msgs::TransformStamped l_rlMap;
    geometry_msgs::TransformStamped l_rrMap;
    try{
        l_flMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME, "FL_foot", ros::Time(0));
        l_frMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME, "FR_foot", ros::Time(0));
        l_rlMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME, "RL_foot", ros::Time(0));
        l_rrMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME, "RR_foot", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Planner: Could not transform feet poses from CoM frame to map frame.");
        return;
    }

    // Populate map feet poses entry
    p_feetConfiguration.flMap.x = l_flMap.transform.translation.x;
    p_feetConfiguration.flMap.y = l_flMap.transform.translation.y;
    p_feetConfiguration.frMap.x = l_frMap.transform.translation.x;
    p_feetConfiguration.frMap.y = l_frMap.transform.translation.y;
    p_feetConfiguration.rlMap.x = l_rlMap.transform.translation.x;
    p_feetConfiguration.rlMap.y = l_rlMap.transform.translation.y;
    p_feetConfiguration.rrMap.x = l_rrMap.transform.translation.x;
    p_feetConfiguration.rrMap.y = l_rrMap.transform.translation.y;

    // Populate CoM feet poses entry
    p_feetConfiguration.flCoM.x = l_highState->footPosition2Body[1].x;
    p_feetConfiguration.flCoM.y = l_highState->footPosition2Body[1].y;
    p_feetConfiguration.frCoM.x = l_highState->footPosition2Body[0].x;
    p_feetConfiguration.frCoM.y = l_highState->footPosition2Body[0].y;
    p_feetConfiguration.rlCoM.x = l_highState->footPosition2Body[3].x;
    p_feetConfiguration.rlCoM.y = l_highState->footPosition2Body[3].y;
    p_feetConfiguration.rrCoM.x = l_highState->footPosition2Body[2].x;
    p_feetConfiguration.rrCoM.y = l_highState->footPosition2Body[2].y;

//    ROS_INFO_STREAM("Planner: CoM (MAP) " << p_robotPose->pose.pose.position.x << ", " << p_robotPose->pose.pose.position.y);
//
//    ROS_INFO_STREAM("Planner: FL (MAP) " << l_flMap.transform.translation.x << ", " << l_flMap.transform.translation.x);
//    ROS_INFO_STREAM("Planner: FL (COM) " << p_feetConfiguration.flCoM.x << ", " << p_feetConfiguration.flCoM.y << "\n");
//
//    ROS_INFO_STREAM("Planner: FR (MAP) " << l_frMap.transform.translation.x << ", " << l_frMap.transform.translation.y);
//    ROS_INFO_STREAM("Planner: FL (COM) " << p_feetConfiguration.frCoM.x << ", " << p_feetConfiguration.frCoM.y << "\n");
//
//    ROS_INFO_STREAM("Planner: RL (MAP) " << l_rlMap.transform.translation.x << ", " << l_rlMap.transform.translation.y);
//    ROS_INFO_STREAM("Planner: RL (COM) " << p_feetConfiguration.rlCoM.x << ", " << p_feetConfiguration.rlCoM.y << "\n");
//
//    ROS_INFO_STREAM("Planner: RR (MAP) " << l_rrMap.transform.translation.x << ", " << l_rrMap.transform.translation.y);
//    ROS_INFO_STREAM("Planner: RR (COM) " << p_feetConfiguration.rrCoM.x << ", " << p_feetConfiguration.rrCoM.y << "\n");
//
//    ros::Duration(5).sleep();

    p_feetConfiguration.fr_rl_swinging = p_swingingFRRL;
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

    // Latest odometry pose of t265
    boost::shared_ptr<nav_msgs::Odometry const> l_t265Pose = m_robotPoseCache.getElemBeforeTime(ros::Time::now());
    
    // Offset t265 pose to obtain CoM pose
    nav_msgs::Odometry l_robotPose = *l_t265Pose;
    l_robotPose.pose.pose.position.x -= 0.33118;
    l_robotPose.pose.pose.position.z += 0.0045;

    // Create World3D start object
    tf2::Quaternion l_startPositionQuaternion;
    tf2::convert(l_robotPose.pose.pose.orientation, l_startPositionQuaternion);
    World3D l_worldStartPosition{l_robotPose.pose.pose.position.x,
                                 l_robotPose.pose.pose.position.y,
                                 l_robotPose.pose.pose.position.z,
                                 l_startPositionQuaternion};

    // Create World3D goal object
    tf2::Quaternion l_goalPositionQuaternion;
    tf2::convert(p_goalPosition.pose.orientation, l_goalPositionQuaternion);
    World3D l_worldGoalPosition{p_goalPosition.pose.position.x,
                                p_goalPosition.pose.position.y,
                                0,
                                l_goalPositionQuaternion};

    ROS_INFO_STREAM("Received goal pose: " << p_goalPosition.pose.position.x << ", "
                                           << p_goalPosition.pose.position.y << ", "
                                           << p_goalPosition.pose.position.z);
    ROS_INFO_STREAM("Current robot pose: " << l_robotPose.pose.pose.position.x << ", "
                                           << l_robotPose.pose.pose.position.y);

    // Create FeetConfiguration object
    FeetConfiguration l_feetConfiguration;
    getFeetConfiguration(l_robotPose, l_feetConfiguration, p_swingingFRRL);

    // Clear reference path
    p_path.clear();

    // Call the A* search algorithm
    m_search.findPath(p_initialAction,
                      p_initialVelocity,
                      l_worldStartPosition,
                      l_worldGoalPosition,
                      l_robotPose.twist.twist,
                      l_feetConfiguration,
                      p_path);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    m_runtimes.push_back(duration.count());
}

/**
 * Prints highest, lowest and
 * average planner runtime.
 */
void Planner::stats() {
    if (m_runtimes.empty()) {
        ROS_WARN("Planner: Runtime vector is empty... something went wrong");
        return;
    }

    unsigned int l_maxRuntime = *std::max_element(m_runtimes.begin(), m_runtimes.end());
    unsigned int l_minRuntime = *std::min_element(m_runtimes.begin(), m_runtimes.end());
    double l_avgRuntime = std::accumulate(m_runtimes.begin(), m_runtimes.end(), 0.0) / m_runtimes.size();
    ROS_INFO_STREAM("Planner: The highest planner runtime was: " << l_maxRuntime);
    ROS_INFO_STREAM("Planner: The lowest planner runtime was: " << l_minRuntime);
    ROS_INFO_STREAM("Planner: The average planner runtime was: " << l_avgRuntime);
}