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
        m_robotPoseCacheSize(ROBOT_POSE_CACHE_SIZE)
{
    initialize();
}

/**
 * Destructor
 */
Navigation::~Navigation()
{
    m_latestRobotPose.reset();
}

/**
 * Navigation initialization
 */
void Navigation::initialize()
{
    // Acquire initial height map
    if (ACQUIRE_INITIAL_HEIGHT_MAP) buildInitialHeightMap();

    // Target goal subscriber
    m_goalSubscriber = m_nh.subscribe("goal", 10, &Navigation::planHeightMapPath, this);

    // Velocity command publisher
    m_velocityPublisher = m_nh.advertise<geometry_msgs::Twist>(VELOCITY_CMD_TOPIC, 1);

    // Height map service response publisher
    m_heightMapPublisher = m_nh.advertise<grid_map_msgs::GridMap>(HEIGHT_MAP_SERVICE_TOPIC, 1);

    // Robot pose subscriber and cache setup
    m_robotPoseSubscriber.subscribe(m_nh, ROBOT_POSE_TOPIC, 1);
    m_robotPoseCache.connectInput(m_robotPoseSubscriber);
    m_robotPoseCache.setCacheSize(m_robotPoseCacheSize);

    // Height map service client
    m_heightMapServiceClient = m_nh.serviceClient<grid_map_msgs::GetGridMap>("/elevation_mapping/get_raw_submap");
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
    // Get latest robot pose from the cache
    const ros::Time l_latestPoseTime = ros::Time::now();
    m_latestRobotPose = m_robotPoseCache.getElemBeforeTime(l_latestPoseTime);

    // Check that returned robot pose pointer is not NULL
    if (!m_latestRobotPose) return;

    // Transform robot pose from robot frame to map frame
    geometry_msgs::PointStamped l_robotPoseMapFrame;
    try
    {
        geometry_msgs::PointStamped l_robotPoseRobotFrame;
        l_robotPoseRobotFrame.header = m_latestRobotPose->header;
        l_robotPoseRobotFrame.point.x = m_latestRobotPose->pose.pose.position.x;
        l_robotPoseRobotFrame.point.y = m_latestRobotPose->pose.pose.position.y;
        l_robotPoseRobotFrame.point.z = m_latestRobotPose->pose.pose.position.z;

        m_buffer.transform(l_robotPoseRobotFrame, l_robotPoseMapFrame, HEIGHT_MAP_REFERENCE_FRAME, ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Navigation: Could not transform robot pose to map frame. Skipping this iteration.");
        return;
    }

    // Service message passed to height map service
    grid_map_msgs::GetGridMap l_heightMapSrv;
    l_heightMapSrv.request.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    l_heightMapSrv.request.position_x = l_robotPoseMapFrame.point.x;
    l_heightMapSrv.request.position_y = l_robotPoseMapFrame.point.y;
    l_heightMapSrv.request.length_x = HEIGHT_MAP_LENGTH_X;
    l_heightMapSrv.request.length_y = HEIGHT_MAP_LENGTH_Y;
    l_heightMapSrv.request.layers.push_back(std::string("elevation"));

    ROS_INFO_STREAM(l_heightMapSrv.request);

    // Call height map service request
    grid_map_msgs::GridMap l_heightMap;
    if (m_heightMapServiceClient.call(l_heightMapSrv))
    {
        l_heightMap = l_heightMapSrv.response.map;
        m_heightMapPublisher.publish(l_heightMap);
        ROS_INFO("Navigation: Height map service call successful.");
    }
    else
    {
        ROS_ERROR("Navigation: Failed to call height map service call");
        return;
    }

    // Call planner to find path to goal
    m_planner.plan(l_heightMap, l_robotPoseMapFrame, p_goalMsg);

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

