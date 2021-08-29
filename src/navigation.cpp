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
        m_planner(p_nh),
        m_buffer(p_buffer)
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
    std::vector<World2D> l_path;
    m_planner.plan(p_goalMsg, l_path);

    // Publish path
    if (!l_path.empty())
    {
        // Path message to be published
        nav_msgs::Path l_pathMsg;
        l_pathMsg.header.stamp = ros::Time::now();
        l_pathMsg.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;

        // Populate path message
        for (auto &l_worldCoordinate: l_path)
        {
            geometry_msgs::PoseStamped l_poseStamped;
            l_poseStamped.header = l_pathMsg.header;
            l_poseStamped.pose.position.x = l_worldCoordinate.x;
            l_poseStamped.pose.position.y = l_worldCoordinate.y;
            l_pathMsg.poses.push_back(l_poseStamped);
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

