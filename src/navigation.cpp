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
 * @param nh
 */
Navigation::Navigation(ros::NodeHandle& p_nh, tf2_ros::Buffer &p_buffer, tf2_ros::TransformListener &p_tf2):
        m_nh(p_nh),
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

    // Path publisher
    m_pathPublisher = m_nh.advertise<nav_msgs::Path>(PATH_TOPIC, 1);

    // Velocity command publisher
    m_velocityPublisher = m_nh.advertise<geometry_msgs::Twist>(VELOCITY_CMD_TOPIC, 1);

    // Feet configuration marker array publisher
    m_feetConfigurationPublisher = m_nh.advertise<visualization_msgs::MarkerArray>(FEET_CONFIGURATION_MARKERS_TOPIC, 1);

    // Target goal subscriber
    m_goalSubscriber = m_nh.subscribe("goal", 10, &Navigation::planHeightMapPath, this);
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

    // Publish path
    if (!l_path.empty())
    {
        // Path message to be published
        nav_msgs::Path l_pathMsg;
        l_pathMsg.header.stamp = ros::Time::now();
        l_pathMsg.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;

        // Populate messages
        int j = 0;
        for (auto &l_node: l_path)
        {
            // Feet configuration array
            // for visualization purposes
            visualization_msgs::MarkerArray l_pathFeetConfiguration;

            // World coordinates for the path
            geometry_msgs::PoseStamped l_poseStamped;
            l_poseStamped.header = l_pathMsg.header;
            l_poseStamped.pose.position.x = l_node.worldCoordinates.x;
            l_poseStamped.pose.position.y = l_node.worldCoordinates.y;
            l_pathMsg.poses.push_back(l_poseStamped);

            // Node in map frame
            geometry_msgs::PointStamped l_CoMMapFrame;
            l_CoMMapFrame.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
            l_CoMMapFrame.header.stamp = ros::Time::now();
            l_CoMMapFrame.point.x = l_node.worldCoordinates.x;
            l_CoMMapFrame.point.y = l_node.worldCoordinates.y;
            l_CoMMapFrame.point.z = 0;

            // Transform node from map to CoM FRAME
            geometry_msgs::PointStamped l_CoMRobotFrame;
            try
            {
                m_buffer.transform(l_CoMMapFrame, l_CoMRobotFrame, ROBOT_REFERENCE_FRAME, ros::Duration(1.0));
                ROS_DEBUG_STREAM("CoM Map Frame: " << l_CoMMapFrame.point.x << ", " << l_CoMRobotFrame.point.y);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("Navigation: Could not transform node from map frame to CoM frame. Returning.");
                return;
            }

            // CoM rotation matrix w.r.t to Map frame
            geometry_msgs::TransformStamped l_rotationTransform;
            l_rotationTransform.header.stamp = ros::Time::now();
            l_rotationTransform.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
            l_rotationTransform.transform.translation.x = 0;
            l_rotationTransform.transform.translation.y = 0;
            l_rotationTransform.transform.translation.z = 0;
            l_rotationTransform.transform.rotation.x = l_node.worldCoordinates.q.x();
            l_rotationTransform.transform.rotation.y = l_node.worldCoordinates.q.y();
            l_rotationTransform.transform.rotation.z = l_node.worldCoordinates.q.z();
            l_rotationTransform.transform.rotation.w = l_node.worldCoordinates.q.w();

            // Apply rotation matrix to feet configuration translation matrix
            geometry_msgs::PointStamped l_flFootConfiguration;
            l_flFootConfiguration.header.stamp = ros::Time::now();
            l_flFootConfiguration.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
            l_flFootConfiguration.point.x = l_node.feetConfiguration.fl.x;
            l_flFootConfiguration.point.y = l_node.feetConfiguration.fl.y;
            l_flFootConfiguration.point.z = 0;

            geometry_msgs::PointStamped l_frFootConfiguration;
            l_frFootConfiguration.header = l_flFootConfiguration.header;
            l_frFootConfiguration.point.x = l_node.feetConfiguration.fr.x;
            l_frFootConfiguration.point.y = l_node.feetConfiguration.fr.y;
            l_frFootConfiguration.point.z = 0;

            geometry_msgs::PointStamped l_rlFootConfiguration;
            l_rlFootConfiguration.header = l_flFootConfiguration.header;
            l_rlFootConfiguration.point.x = l_node.feetConfiguration.rl.x;
            l_rlFootConfiguration.point.y = l_node.feetConfiguration.rl.y;
            l_rlFootConfiguration.point.z = 0;

            geometry_msgs::PointStamped l_rrFootConfiguration;
            l_rrFootConfiguration.header = l_flFootConfiguration.header;
            l_rrFootConfiguration.point.x = l_node.feetConfiguration.rr.x;
            l_rrFootConfiguration.point.y = l_node.feetConfiguration.rr.y;
            l_rrFootConfiguration.point.z = 0;

            geometry_msgs::PointStamped l_flFootConfigurationRotated;
            geometry_msgs::PointStamped l_frFootConfigurationRotated;
            geometry_msgs::PointStamped l_rlFootConfigurationRotated;
            geometry_msgs::PointStamped l_rrFootConfigurationRotated;
            tf2::doTransform(l_flFootConfiguration, l_flFootConfigurationRotated, l_rotationTransform);
            tf2::doTransform(l_frFootConfiguration, l_frFootConfigurationRotated, l_rotationTransform);
            tf2::doTransform(l_rlFootConfiguration, l_rlFootConfigurationRotated, l_rotationTransform);
            tf2::doTransform(l_rrFootConfiguration, l_rrFootConfigurationRotated, l_rotationTransform);

            ROS_INFO_STREAM("Action: " << l_node.action.x * l_node.velocity << ", "
                                            << l_node.action.y * l_node.velocity << ", "
                                            << l_node.action.theta * l_node.velocity);
            ROS_INFO_STREAM("Predicted FL position: " << l_node.feetConfiguration.fl.x << ", "
                                                           << l_node.feetConfiguration.fl.y);
            ROS_INFO_STREAM("Predicted FR position: " << l_node.feetConfiguration.fr.x << ", "
                                                           << l_node.feetConfiguration.fr.y);
            ROS_INFO_STREAM("Predicted RL position: " << l_node.feetConfiguration.rl.x << ", "
                                                           << l_node.feetConfiguration.rl.y);
            ROS_INFO_STREAM("Predicted RR position: " << l_node.feetConfiguration.rr.x << ", "
                                                           << l_node.feetConfiguration.rr.y << "\n");

            // Populate array
            visualization_msgs::Marker l_footCommonMarker;
            l_footCommonMarker.header.stamp = ros::Time::now();
            l_footCommonMarker.header.frame_id = ROBOT_REFERENCE_FRAME;
            l_footCommonMarker.type = 2;
            l_footCommonMarker.action = 0;
            l_footCommonMarker.lifetime = ros::Duration(1);
            l_footCommonMarker.pose.orientation.x = 0;
            l_footCommonMarker.pose.orientation.y = 0;
            l_footCommonMarker.pose.orientation.z = 0;
            l_footCommonMarker.pose.orientation.w = 1;
            l_footCommonMarker.scale.x = 0.025;
            l_footCommonMarker.scale.y = 0.025;
            l_footCommonMarker.scale.z = 0.025;
            l_footCommonMarker.color.r = 0.77;
            l_footCommonMarker.color.g = 0.08;
            l_footCommonMarker.color.b = 0.52;
            l_footCommonMarker.color.a = 0.5;

            visualization_msgs::Marker l_CoMMarker = l_footCommonMarker;
            l_CoMMarker.id = j++;
            l_CoMMarker.pose.position.x = l_CoMRobotFrame.point.x;
            l_CoMMarker.pose.position.y = l_CoMRobotFrame.point.y;
            l_CoMMarker.pose.position.z = 0;

            visualization_msgs::Marker l_flFootMarker = l_footCommonMarker;
            l_flFootMarker.id = j++;
            l_flFootMarker.pose.position.x = l_CoMRobotFrame.point.x + l_flFootConfigurationRotated.point.x;
            l_flFootMarker.pose.position.y = l_CoMRobotFrame.point.y + l_flFootConfigurationRotated.point.y;
            l_flFootMarker.pose.position.z = 0;

            visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
            l_frFootMarker.id = j++;
            l_frFootMarker.pose.position.x = l_CoMRobotFrame.point.x + l_frFootConfigurationRotated.point.x;
            l_frFootMarker.pose.position.y = l_CoMRobotFrame.point.y + l_frFootConfigurationRotated.point.y;
            l_frFootMarker.pose.position.z = 0;

            visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
            l_rlFootMarker.id = j++;
            l_rlFootMarker.pose.position.x = l_CoMRobotFrame.point.x + l_rlFootConfigurationRotated.point.x;
            l_rlFootMarker.pose.position.y = l_CoMRobotFrame.point.y + l_rlFootConfigurationRotated.point.y;
            l_rlFootMarker.pose.position.z = 0;

            visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
            l_rrFootMarker.id = j++;
            l_rrFootMarker.pose.position.x = l_CoMRobotFrame.point.x + l_rrFootConfigurationRotated.point.x;
            l_rrFootMarker.pose.position.y = l_CoMRobotFrame.point.y + l_rrFootConfigurationRotated.point.y;
            l_rrFootMarker.pose.position.z = 0;

            l_pathFeetConfiguration.markers.push_back(l_CoMMarker);
            l_pathFeetConfiguration.markers.push_back(l_flFootMarker);
            l_pathFeetConfiguration.markers.push_back(l_frFootMarker);
            l_pathFeetConfiguration.markers.push_back(l_rlFootMarker);
            l_pathFeetConfiguration.markers.push_back(l_rrFootMarker);

            m_feetConfigurationPublisher.publish(l_pathFeetConfiguration);
            ros::Duration(1).sleep();
        }

        m_pathPublisher.publish(l_pathMsg);

        ROS_INFO("Navigation: Path obtained and published.");
    }
    else
    {
        ROS_WARN("Navigation: Path obtained is empty.");
    }

    //TODO: apply footsteps
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

    // Spin
    ros::spin();
    return 0;
}

