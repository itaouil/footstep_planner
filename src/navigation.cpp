/*
 * navigation.cpp
 *
 *  Created on: Aug 16, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#include "navigation.hpp"

/**
 * Constructor
 *
 * @param p_nh
 * @param p_buffer
 * @param p_tf2
 */
Navigation::Navigation(ros::NodeHandle& p_nh, tf2_ros::Buffer &p_buffer, tf2_ros::TransformListener &p_tf2):
        m_nh(p_nh),
        m_rate(100),
        m_tf2(p_tf2),
        m_buffer(p_buffer),
        m_planner(p_nh)
{
    initialize();
}

/**
 * Destructor
 */
Navigation::~Navigation() = default;

/**
 * Navigation initialization
 */
void Navigation::initialize()
{
    // Acquire initial height map
    if (ACQUIRE_INITIAL_HEIGHT_MAP) buildInitialHeightMap();

    // Robot pose subscriber and cache setup
    m_robotPoseSubscriber.subscribe(m_nh, ROBOT_POSE_TOPIC, 1);
    m_robotPoseCache.connectInput(m_robotPoseSubscriber);
    m_robotPoseCache.setCacheSize(CACHE_SIZE);

    // Path publishers
    m_realPathPublisher = m_nh.advertise<nav_msgs::Path>(REAL_PATH_TOPIC, 1);
    m_targetPathPublisher = m_nh.advertise<nav_msgs::Path>(TARGET_PATH_TOPIC, 1);

    // Velocity command publisher
    m_velocityPublisher = m_nh.advertise<geometry_msgs::Twist>(VELOCITY_CMD_TOPIC, 10);

    // Target goal subscriber
    m_goalSubscriber = m_nh.subscribe("goal", 10, &Navigation::planHeightMapPath, this);

    // Feet configuration marker array publisher
    m_targetFeetConfigurationPublisher = m_nh.advertise<visualization_msgs::MarkerArray>(TARGET_FEET_CONFIGURATION_MARKERS_TOPIC, 1);
}

/**
 * Performs a 360 rotation to acquire
 * full height map to be used for planning
 */
void Navigation::buildInitialHeightMap()
{
    ROS_INFO("Navigation: Started rotation behaviour to acquire full height map");

    // Const values
    const int l_angle = 360;
    const double l_speed = 10;

    // Convert from angles to radians
    const double l_angularSpeed = l_speed * 2 * M_PI / 360;
    const double l_relativeAngle = l_angle * 2 * M_PI / 360;

    // Build Twist message to send
    geometry_msgs::Twist l_velocityMsg;
    l_velocityMsg.linear.x = 0;
    l_velocityMsg.linear.y = 0;
    l_velocityMsg.linear.z = 0;
    l_velocityMsg.angular.x = 0;
    l_velocityMsg.angular.y = 0;
    l_velocityMsg.angular.z = l_angularSpeed;

    // Setting current variables for distance calculus
    double l_currentAngle = 0;
    double l_t0 = ros::Time::now().toSec();

    // Send velocity command
    while (l_currentAngle < l_relativeAngle)
    {
        // Publish message
        m_velocityPublisher.publish(l_velocityMsg);

        // Compute current robot direction
        double l_t1 = ros::Time::now().toSec();
        l_currentAngle = l_angularSpeed * (l_t1 - l_t0);

        ros::spinOnce();
    }

    // Stop sending velocity commands
    l_velocityMsg.angular.z = 0;
    m_velocityPublisher.publish(l_velocityMsg);

    ROS_INFO("Navigation: Rotation behaviour completed.");
}

/**
 * Plans footsteps location using
 * the most up to date height map
 */
void Navigation::planHeightMapPath(const geometry_msgs::PoseStamped &p_goalMsg)
{
    ROS_INFO("Navigation: Planning request received.");

    // Call planner to find path to goal
    std::vector<Node> l_path;
    m_planner.plan(p_goalMsg, l_path);

    // Publish path + execute commands
    if (!l_path.empty())
    {
        // Path message to be published
        nav_msgs::Path l_pathMsg;
        l_pathMsg.header.stamp = ros::Time::now();
        l_pathMsg.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;

        // Publish path
        for (auto &l_node: l_path)
        {
            // World coordinates for the path
            geometry_msgs::PoseStamped l_poseStamped;
            l_poseStamped.header = l_pathMsg.header;
            l_poseStamped.pose.position.x = l_node.worldCoordinates.x;
            l_poseStamped.pose.position.y = l_node.worldCoordinates.y;
            l_poseStamped.pose.position.z = l_node.worldCoordinates.z;
            l_pathMsg.poses.push_back(l_poseStamped);

            // Create twist command
            geometry_msgs::Twist l_velocityMsg;
            l_velocityMsg.linear.x = l_node.action.x * l_node.velocity;
            l_velocityMsg.linear.y = l_node.action.y * l_node.velocity;
            l_velocityMsg.linear.z = 0;
            l_velocityMsg.angular.x = 0;
            l_velocityMsg.angular.y = 0;
            l_velocityMsg.angular.z = l_node.action.theta * l_node.velocity;

            // Populate motion command queue
            m_motionCommands.push(l_velocityMsg);

            ROS_INFO_STREAM("Velocity; " << l_node.velocity);
            ROS_INFO_STREAM("Swinging foot: " << l_node.feetConfiguration.fr_rl_swinging);
            ROS_INFO_STREAM("Action: " << l_node.action.x << ", " << l_node.action.y << ", " << l_node.action.theta);
        }
        m_targetPathPublisher.publish(l_pathMsg);

        publishPredictedFootstepSequence(l_path);

        // Real path message to be published
//        nav_msgs::Path l_realPathMsg;
//        l_realPathMsg.header.stamp = ros::Time::now();
//        l_realPathMsg.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
//
//        int count = 0;
//        Action l_prevAction{};
//        for (auto &l_node: l_path)
//        {
//            if (count == 0)
//            {
//                l_prevAction = l_node.action;
//            }
//
//            // Create twist command
//            geometry_msgs::Twist l_velocityMsg;
//            l_velocityMsg.linear.x = l_node.action.x * l_node.velocity;
//            l_velocityMsg.linear.y = l_node.action.y * l_node.velocity;
//            l_velocityMsg.linear.z = 0;
//            l_velocityMsg.angular.x = 0;
//            l_velocityMsg.angular.y = 0;
//            l_velocityMsg.angular.z = l_node.action.theta * l_node.velocity;
//
//            // Get the latest feet poses
//            ros::Time l_latestTime = ros::Time::now();
//            boost::shared_ptr<nav_msgs::Odometry const> l_latestOdomPose =
//                    m_robotPoseCache.getElemBeforeTime(l_latestTime);
//
//            // Get target yaw
//            double l_roll, l_pitch, l_yaw;
//            tf2::Matrix3x3 l_rotationMatrix(l_node.worldCoordinates.q);
//            l_rotationMatrix.getRPY(l_roll, l_pitch, l_yaw);
//            double l_targetYaw = ((l_yaw * 180) / M_PI);
//
//            // Get current yaw
//            tf2::Quaternion q(l_latestOdomPose->pose.pose.orientation.x,
//                              l_latestOdomPose->pose.pose.orientation.y,
//                              l_latestOdomPose->pose.pose.orientation.z,
//                              l_latestOdomPose->pose.pose.orientation.w);
//            tf2::Matrix3x3 l_rotationMatrix2(q);
//            l_rotationMatrix2.getRPY(l_roll, l_pitch, l_yaw);
//            double l_currentYaw = ((l_yaw * 180) / M_PI);
//
//            ROS_INFO_STREAM("Command: " << count << "/" << l_path.size());
//
//            if (count != 0 && l_prevAction != l_node.action)
//                ros::Duration(0.5).sleep();
//
//            // Amount of time for time execution
//            double l_stopTime = ros::Time::now().toSec() + ros::Duration(0.55).toSec();
//
//            // Execute command for rotation motions
//            if (l_node.action == Action{0, 0, 1} || l_node.action == Action{0, 0, -1})
//            {
//                while (std::abs(l_currentYaw - l_targetYaw) > 5 && ros::Time::now().toSec() < l_stopTime)
//                {
//                    // Publish motion command
//                    m_velocityPublisher.publish(l_velocityMsg);
//
//                    // Update foot pose from cache
//                    l_latestTime = ros::Time::now();
//                    l_latestOdomPose = m_robotPoseCache.getElemBeforeTime(l_latestTime);
//
//                    // World coordinates for the path
//                    geometry_msgs::PoseStamped l_realPoseStamped;
//                    l_realPoseStamped.header = l_realPathMsg.header;
//                    l_realPoseStamped.pose.position.x = l_latestOdomPose->pose.pose.position.x;
//                    l_realPoseStamped.pose.position.y = l_latestOdomPose->pose.pose.position.y;
//                    l_realPathMsg.poses.push_back(l_realPoseStamped);
//
//                    // Update current yaw
//                    tf2::Quaternion q_t(l_latestOdomPose->pose.pose.orientation.x,
//                                        l_latestOdomPose->pose.pose.orientation.y,
//                                        l_latestOdomPose->pose.pose.orientation.z,
//                                        l_latestOdomPose->pose.pose.orientation.w);
//                    tf2::Matrix3x3 l_rotationMatrix_t(q_t);
//                    l_rotationMatrix_t.getRPY(l_roll, l_pitch, l_yaw);
//                    l_currentYaw = ((l_yaw * 180) / M_PI);
//
//                    m_realPathPublisher.publish(l_realPathMsg);
//                    m_targetFeetConfigurationPublisher.publish(m_targetFootsteps[count]);
//
//                    // Get callback data
//                    ros::spinOnce();
//
////                    // Sleep
////                    m_rate.sleep();
//                }
//            }
//            else
//            {
//                while ((std::abs(l_latestOdomPose->pose.pose.position.x - l_node.worldCoordinates.x) > 0.005 ||
//                       std::abs(l_latestOdomPose->pose.pose.position.y - l_node.worldCoordinates.y) > 0.005) &&
//                       ros::Time::now().toSec() < l_stopTime)
//                {
//                    // Publish motion command
//                    m_velocityPublisher.publish(l_velocityMsg);
//
////                    // Sleep
////                    m_rate.sleep();
//
//                    // Update foot pose from cache
//                    l_latestTime = ros::Time::now();
//                    l_latestOdomPose = m_robotPoseCache.getElemBeforeTime(l_latestTime);
//
//                    // World coordinates for the path
//                    geometry_msgs::PoseStamped l_realPoseStamped;
//                    l_realPoseStamped.header = l_realPathMsg.header;
//                    l_realPoseStamped.pose.position.x = l_latestOdomPose->pose.pose.position.x;
//                    l_realPoseStamped.pose.position.y = l_latestOdomPose->pose.pose.position.y;
//                    l_realPathMsg.poses.push_back(l_realPoseStamped);
//
//                    m_realPathPublisher.publish(l_realPathMsg);
//                    m_targetFeetConfigurationPublisher.publish(m_targetFootsteps[count]);
//
//                    // Get callback data
//                    ros::spinOnce();
//                }
//            }
//
//            count += 1;
//            l_prevAction = l_node.action;
//        }
    }
    else
    {
        ROS_WARN("Navigation: Path obtained is empty.");
    }

    //TODO: apply footsteps
}

void Navigation::publishPredictedFootstepSequence(const std::vector<Node> &p_path)
{
    int j = 0;
    ROS_INFO_STREAM("Path size: " << p_path.size());

    for (auto &l_node: p_path)
    {
        // Feet configuration array
        visualization_msgs::MarkerArray l_targetFeetConfiguration;
        visualization_msgs::MarkerArray l_targetFeetConfigurationOverlay;

        // Populate array
        visualization_msgs::Marker l_footCommonMarker;
        l_footCommonMarker.header.stamp = ros::Time::now();
        l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
        l_footCommonMarker.type = 2;
        l_footCommonMarker.action = 0;
        l_footCommonMarker.lifetime = ros::Duration(0.34);
        l_footCommonMarker.pose.orientation.x = 0;
        l_footCommonMarker.pose.orientation.y = 0;
        l_footCommonMarker.pose.orientation.z = 0;
        l_footCommonMarker.pose.orientation.w = 1;
        l_footCommonMarker.scale.x = 0.035;
        l_footCommonMarker.scale.y = 0.035;
        l_footCommonMarker.scale.z = 0.035;
        l_footCommonMarker.color.r = 0;
        l_footCommonMarker.color.g = 0;
        l_footCommonMarker.color.b = 0;
        l_footCommonMarker.color.a = 1;

        visualization_msgs::Marker l_CoMMarker = l_footCommonMarker;
        l_CoMMarker.id = j++;
        l_CoMMarker.pose.position.x = l_node.worldCoordinates.x;
        l_CoMMarker.pose.position.y = l_node.worldCoordinates.y;
        l_CoMMarker.pose.position.z = l_node.worldCoordinates.z;

        visualization_msgs::Marker l_flFootMarker = l_footCommonMarker;
        l_flFootMarker.id = j++;
        l_flFootMarker.pose.position.x = l_node.feetConfiguration.flMap.x;
        l_flFootMarker.pose.position.y = l_node.feetConfiguration.flMap.y;
        l_flFootMarker.pose.position.z = l_node.feetConfiguration.flMap.z;

        visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
        l_frFootMarker.id = j++;
        l_frFootMarker.pose.position.x = l_node.feetConfiguration.frMap.x;
        l_frFootMarker.pose.position.y = l_node.feetConfiguration.frMap.y;
        l_frFootMarker.pose.position.z = l_node.feetConfiguration.frMap.z;

        visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
        l_rlFootMarker.id = j++;
        l_rlFootMarker.pose.position.x = l_node.feetConfiguration.rlMap.x;
        l_rlFootMarker.pose.position.y = l_node.feetConfiguration.rlMap.y;
        l_rlFootMarker.pose.position.z = l_node.feetConfiguration.rlMap.z;

        visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
        l_rrFootMarker.id = j++;
        l_rrFootMarker.pose.position.x = l_node.feetConfiguration.rrMap.x;
        l_rrFootMarker.pose.position.y = l_node.feetConfiguration.rrMap.y;
        l_rrFootMarker.pose.position.z = l_node.feetConfiguration.rrMap.z;

        l_targetFeetConfiguration.markers.push_back(l_CoMMarker);
        l_targetFeetConfiguration.markers.push_back(l_flFootMarker);
        l_targetFeetConfiguration.markers.push_back(l_frFootMarker);
        l_targetFeetConfiguration.markers.push_back(l_rlFootMarker);
        l_targetFeetConfiguration.markers.push_back(l_rrFootMarker);

        m_targetFeetConfigurationPublisher.publish(l_targetFeetConfiguration);
        ros::Duration(0.38).sleep();

//        l_CoMMarker.lifetime = ros::Duration(0.02);
//        l_CoMMarker.color.a = 0.5;
//        l_flFootMarker.lifetime = ros::Duration(0.02);
//        l_flFootMarker.color.a = 0.5;
//        l_frFootMarker.lifetime = ros::Duration(0.02);
//        l_frFootMarker.color.a = 0.5;
//        l_rlFootMarker.lifetime = ros::Duration(0.02);
//        l_rlFootMarker.color.a = 0.5;
//        l_rrFootMarker.lifetime = ros::Duration(0.02);
//        l_rrFootMarker.color.a = 0.5;
//        l_targetFeetConfigurationOverlay.markers.push_back(l_CoMMarker);
//        l_targetFeetConfigurationOverlay.markers.push_back(l_flFootMarker);
//        l_targetFeetConfigurationOverlay.markers.push_back(l_frFootMarker);
//        l_targetFeetConfigurationOverlay.markers.push_back(l_rlFootMarker);
//        l_targetFeetConfigurationOverlay.markers.push_back(l_rrFootMarker);
//        m_targetFootsteps.push_back(l_targetFeetConfigurationOverlay);
    }
}

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "aliengo_navigation");
    ros::NodeHandle nodeHandle("~");

    // TF2 objects
    tf2_ros::Buffer l_buffer(ros::Duration(10));
    tf2_ros::TransformListener l_tf(l_buffer);

    // Start navigation
    Navigation navigation(nodeHandle, l_buffer, l_tf);

    // Use two threads
    ros::AsyncSpinner l_spinner(2);
    l_spinner.start();

    ros::waitForShutdown();

    return 0;
}