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
        m_listener(m_buffer) {}

Planner::~Planner() = default;

/**
 * Populate feet configuration struct
 *
 * @param p_swingingFRRL
 * @param p_robotPose
 * @param p_feetConfiguration
 * @param p_latestCoMFeetPoses
 */
void Planner::getFeetConfiguration(const bool &p_swingingFRRL,
                                   const nav_msgs::Odometry &p_robotPose,
                                   FeetConfiguration &p_feetConfiguration,
                                   const std::vector<unitree_legged_msgs::Cartesian> &p_latestCoMFeetPoses) {
    geometry_msgs::TransformStamped lfFootPoseMap;
    geometry_msgs::TransformStamped rfFootPoseMap;
    geometry_msgs::TransformStamped lhFootPoseMap;
    geometry_msgs::TransformStamped rhFootPoseMap;
    try{
        lfFootPoseMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME,
                                                 LF_FOOT_REFERENCE_FRAME,
                                                 ros::Time(0));
        rfFootPoseMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME,
                                                 RF_FOOT_REFERENCE_FRAME,
                                                 ros::Time(0));
        lhFootPoseMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME,
                                                 LH_FOOT_REFERENCE_FRAME,
                                                 ros::Time(0));
        rhFootPoseMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME,
                                                 RH_FOOT_REFERENCE_FRAME,
                                                 ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Could not get transform for feet in world frame: ",ex.what());
    }

    // Feet poses w.r.t map
    p_feetConfiguration.flMap.x = lfFootPoseMap.transform.translation.x;
    p_feetConfiguration.flMap.y = lfFootPoseMap.transform.translation.y;
    p_feetConfiguration.frMap.x = rfFootPoseMap.transform.translation.x;
    p_feetConfiguration.frMap.y = rfFootPoseMap.transform.translation.y;
    p_feetConfiguration.rlMap.x = lhFootPoseMap.transform.translation.x;
    p_feetConfiguration.rlMap.y = lhFootPoseMap.transform.translation.y;
    p_feetConfiguration.rrMap.x = rhFootPoseMap.transform.translation.x;
    p_feetConfiguration.rrMap.y = rhFootPoseMap.transform.translation.y;

    geometry_msgs::TransformStamped lfFootPoseCoM;
    geometry_msgs::TransformStamped rfFootPoseCoM;
    geometry_msgs::TransformStamped lhFootPoseCoM;
    geometry_msgs::TransformStamped rhFootPoseCoM;
    try{
        lfFootPoseCoM = m_buffer.lookupTransform(ROBOT_REFERENCE_FRAME,
                                                 LF_FOOT_REFERENCE_FRAME,
                                                 ros::Time(0));
        rfFootPoseCoM = m_buffer.lookupTransform(ROBOT_REFERENCE_FRAME,
                                                 RF_FOOT_REFERENCE_FRAME,
                                                 ros::Time(0));
        lhFootPoseCoM = m_buffer.lookupTransform(ROBOT_REFERENCE_FRAME,
                                                 LH_FOOT_REFERENCE_FRAME,
                                                 ros::Time(0));
        rhFootPoseCoM = m_buffer.lookupTransform(ROBOT_REFERENCE_FRAME,
                                                 RH_FOOT_REFERENCE_FRAME,
                                                 ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Could not get transform for feet in CoM frame: ", ex.what());
    }

    // Feet poses w.r.t CoM
    p_feetConfiguration.flCoM.x = lfFootPoseCoM.transform.translation.x;
    p_feetConfiguration.flCoM.y = lfFootPoseCoM.transform.translation.y;
    p_feetConfiguration.frCoM.x = rfFootPoseCoM.transform.translation.x;
    p_feetConfiguration.frCoM.y = rfFootPoseCoM.transform.translation.y;
    p_feetConfiguration.rlCoM.x = lhFootPoseCoM.transform.translation.x;
    p_feetConfiguration.rlCoM.y = lhFootPoseCoM.transform.translation.y;
    p_feetConfiguration.rrCoM.x = rhFootPoseCoM.transform.translation.x;
    p_feetConfiguration.rrCoM.y = rhFootPoseCoM.transform.translation.y;

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
 * @param p_path
 * @param p_swingingPair
 * @param p_initialAction
 * @param p_initialVelocity
 * @param p_robotPose
 * @param p_goalPosition
 * @param p_latestCoMFeetPoses
 */
void Planner::plan(std::vector<Node> &p_path,
                   const bool &p_swingingPair,
                   const Action &p_initialAction,
                   const double &p_initialVelocity,
                   const nav_msgs::Odometry &p_robotPose,
                   const geometry_msgs::PoseStamped &p_goalPosition,
                   const std::vector<unitree_legged_msgs::Cartesian> &p_latestCoMFeetPoses) {
    auto start = high_resolution_clock::now();

    // Create World3D start object
    tf2::Quaternion l_startPositionQuaternion;
    tf2::convert(p_robotPose.pose.pose.orientation, l_startPositionQuaternion);
    World3D l_worldStartPosition{p_robotPose.pose.pose.position.x,
                                 p_robotPose.pose.pose.position.y,
                                 p_robotPose.pose.pose.position.z,
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
    ROS_INFO_STREAM("Current robot pose: " << p_robotPose.pose.pose.position.x << ", "
                                           << p_robotPose.pose.pose.position.y);

    // Create FeetConfiguration object
    FeetConfiguration l_feetConfiguration;
    getFeetConfiguration(p_swingingPair, p_robotPose, l_feetConfiguration, p_latestCoMFeetPoses);

    // Clear reference path
    p_path.clear();

    // Call the A* search algorithm
    m_search.findPath(p_initialAction,
                      p_initialVelocity,
                      l_worldStartPosition,
                      l_worldGoalPosition,
                      p_robotPose.twist.twist,
                      l_feetConfiguration,
                      p_path);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
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
    ROS_INFO_STREAM("Planner: The highest planner runtime was: " << l_maxRuntime << " ms.");
    ROS_INFO_STREAM("Planner: The lowest planner runtime was: " << l_minRuntime << " ms.");
    ROS_INFO_STREAM("Planner: The average planner runtime was: " << l_avgRuntime << " ms.");
}