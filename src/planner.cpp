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
    // // Rotation of CoM w.r.t World
    // geometry_msgs::TransformStamped l_R_W_C;
    // l_R_W_C.header.stamp = ros::Time::now();
    // l_R_W_C.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    // l_R_W_C.transform.translation.x = p_robotPose.pose.pose.position.x;
    // l_R_W_C.transform.translation.y = p_robotPose.pose.pose.position.y;
    // l_R_W_C.transform.translation.z = 0.0;
    // l_R_W_C.transform.rotation.x = p_robotPose.pose.pose.orientation.x;
    // l_R_W_C.transform.rotation.y = p_robotPose.pose.pose.orientation.y;
    // l_R_W_C.transform.rotation.z = p_robotPose.pose.pose.orientation.z;
    // l_R_W_C.transform.rotation.w = p_robotPose.pose.pose.orientation.w;

    // // Transform latest CoM feet configuration to map frame 
    // std::vector<double> l_feetPredictionWorldFrame;
    // geometry_msgs::PointStamped l_feetPredictionCoMFrame;
    // l_feetPredictionCoMFrame.header.stamp = ros::Time::now();
    // l_feetPredictionCoMFrame.header.frame_id = ROBOT_REFERENCE_FRAME;
    // for (int x = 0; x < 4; x++) {
    //     l_feetPredictionCoMFrame.point.x = p_latestCoMFeetPoses[x].x;
    //     l_feetPredictionCoMFrame.point.y = p_latestCoMFeetPoses[x].y;
    //     l_feetPredictionCoMFrame.point.z = 0;

    //     geometry_msgs::PointStamped l_footDisplacementWorldFrame;
    //     tf2::doTransform(l_feetPredictionCoMFrame, l_footDisplacementWorldFrame, l_R_W_C);

    //     l_feetPredictionWorldFrame.push_back(l_footDisplacementWorldFrame.point.x);
    //     l_feetPredictionWorldFrame.push_back(l_footDisplacementWorldFrame.point.y);
    // }

    // // Feet poses w.r.t map
    // p_feetConfiguration.flMap.x = l_feetPredictionWorldFrame[0];
    // p_feetConfiguration.flMap.y = l_feetPredictionWorldFrame[1];
    // p_feetConfiguration.frMap.x = l_feetPredictionWorldFrame[2];
    // p_feetConfiguration.frMap.y = l_feetPredictionWorldFrame[3];
    // p_feetConfiguration.rlMap.x = l_feetPredictionWorldFrame[4];
    // p_feetConfiguration.rlMap.y = l_feetPredictionWorldFrame[5];
    // p_feetConfiguration.rrMap.x = l_feetPredictionWorldFrame[6];
    // p_feetConfiguration.rrMap.y = l_feetPredictionWorldFrame[7];

    // ROS_DEBUG_STREAM("Planner: " << p_feetConfiguration.flMap.x << " ," << p_feetConfiguration.flMap.y);
    // ROS_DEBUG_STREAM("Planner: " << p_feetConfiguration.frMap.x << " ," << p_feetConfiguration.frMap.y);

    // // Feet poses w.r.t CoM
    // p_feetConfiguration.flCoM.x = p_latestCoMFeetPoses[0].x;
    // p_feetConfiguration.flCoM.y = p_latestCoMFeetPoses[0].y;
    // p_feetConfiguration.frCoM.x = p_latestCoMFeetPoses[1].x;
    // p_feetConfiguration.frCoM.y = p_latestCoMFeetPoses[1].y;
    // p_feetConfiguration.rlCoM.x = p_latestCoMFeetPoses[2].x;
    // p_feetConfiguration.rlCoM.y = p_latestCoMFeetPoses[2].y;
    // p_feetConfiguration.rrCoM.x = p_latestCoMFeetPoses[3].x;
    // p_feetConfiguration.rrCoM.y = p_latestCoMFeetPoses[3].y;

    geometry_msgs::TransformStamped fl_transform;
    geometry_msgs::TransformStamped fr_transform;
    geometry_msgs::TransformStamped rl_transform;
    geometry_msgs::TransformStamped rr_transform;
    try{
      fl_transform = m_buffer.lookupTransform("world", FEET_FRAMES[0], ros::Time(0));
      fr_transform = m_buffer.lookupTransform("world", FEET_FRAMES[1], ros::Time(0));
      rl_transform = m_buffer.lookupTransform("world", FEET_FRAMES[2], ros::Time(0));
      rr_transform = m_buffer.lookupTransform("world", FEET_FRAMES[3], ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }

    // Feet poses w.r.t map
    p_feetConfiguration.flMap.x = fl_transform.transform.translation.x;
    p_feetConfiguration.flMap.y = fl_transform.transform.translation.y;
    p_feetConfiguration.frMap.x = fr_transform.transform.translation.x;
    p_feetConfiguration.frMap.y = fr_transform.transform.translation.y;
    p_feetConfiguration.rlMap.x = rl_transform.transform.translation.x;
    p_feetConfiguration.rlMap.y = rl_transform.transform.translation.y;
    p_feetConfiguration.rrMap.x = rr_transform.transform.translation.x;
    p_feetConfiguration.rrMap.y = rr_transform.transform.translation.y;

    // Feet poses w.r.t CoM
    p_feetConfiguration.flCoM.x = p_latestCoMFeetPoses[0].x;
    p_feetConfiguration.flCoM.y = p_latestCoMFeetPoses[0].y;
    p_feetConfiguration.frCoM.x = p_latestCoMFeetPoses[1].x;
    p_feetConfiguration.frCoM.y = p_latestCoMFeetPoses[1].y;
    p_feetConfiguration.rlCoM.x = p_latestCoMFeetPoses[2].x;
    p_feetConfiguration.rlCoM.y = p_latestCoMFeetPoses[2].y;
    p_feetConfiguration.rrCoM.x = p_latestCoMFeetPoses[3].x;
    p_feetConfiguration.rrCoM.y = p_latestCoMFeetPoses[3].y;

    p_feetConfiguration.fr_rl_swinging = p_swingingFRRL;
}

/**
 * Plans path from start to goal.
 *
 * @param p_path
 * @param p_swingingPair
 * @param p_initialAction
 * @param p_initialVelocity
 * @param p_previousCoMVelocity
 * @param p_robotPose
 * @param p_goalPosition
 * @param p_latestCoMFeetPoses
 */
void Planner::plan(std::vector<Node> &p_path,
                   const bool &p_swingingPair,
                   const Action &p_initialAction,
                   const double &p_initialVelocity,
                   const double &p_previousCoMVelocity,
                   const nav_msgs::Odometry &p_robotPose,
                   const geometry_msgs::PoseStamped &p_goalPosition,
                   const std::vector<unitree_legged_msgs::Cartesian> &p_latestCoMFeetPoses) {
    auto start = high_resolution_clock::now();

    // Starting position
    tf2::Quaternion l_startPositionQuaternion;
    tf2::convert(p_robotPose.pose.pose.orientation, l_startPositionQuaternion);
    World3D l_worldStartPosition{p_robotPose.pose.pose.position.x,
                                 p_robotPose.pose.pose.position.y,
                                 p_robotPose.pose.pose.position.z,
                                 l_startPositionQuaternion,
                                 p_robotPose.twist.twist.linear.x,
                                 p_previousCoMVelocity};

    // Goal position
    tf2::Quaternion l_goalPositionQuaternion;
    tf2::convert(p_goalPosition.pose.orientation, l_goalPositionQuaternion);
    World3D l_worldGoalPosition{p_goalPosition.pose.position.x,
                                p_goalPosition.pose.position.y,
                                0.0,
                                l_goalPositionQuaternion,
                                0.0,
                                0.0};

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
                      l_feetConfiguration,
                      p_path);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    auto duration_micro = duration_cast<microseconds>(stop - start);
    ROS_INFO_STREAM("Time took to plan: " << duration.count() << " ms");
    ROS_INFO_STREAM("Time took to plan: " << duration_micro.count() << " micro");
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