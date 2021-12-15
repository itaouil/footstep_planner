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
Navigation::Navigation(ros::NodeHandle &p_nh, tf2_ros::Buffer &p_buffer, tf2_ros::TransformListener &p_tf2) :
        m_nh(p_nh),
        m_rate(1000),
        m_tf2(p_tf2),
        m_buffer(p_buffer),
        m_planner(p_nh) {
    initialize();
}

/**
 * Destructor
 */
Navigation::~Navigation() = default;

/**
 * Navigation initialization
 */
void Navigation::initialize() {
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
    m_velocityPublisher = m_nh.advertise<sensor_msgs::Joy>(VELOCITY_CMD_TOPIC, 10);

    // Target goal subscriber
    m_goalSubscriber = m_nh.subscribe("goal", 10, &Navigation::planHeightMapPath, this);

    // Feet configuration marker array publisher
    m_targetFeetConfigurationPublisher = m_nh.advertise<visualization_msgs::MarkerArray>(
            TARGET_FEET_CONFIGURATION_MARKERS_TOPIC, 1);

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

/**
 * Performs a 360 rotation to acquire
 * full height map to be used for planning
 */
void Navigation::buildInitialHeightMap() {
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
    while (l_currentAngle < l_relativeAngle) {
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
 * Publish predicted CoM path.
 *
 * @param p_path
 */
void Navigation::publishPredictedCoMPath(const std::vector<Node> &p_path) {
    // Path message to be published
    nav_msgs::Path l_pathMsg;
    l_pathMsg.header.stamp = ros::Time::now();
    l_pathMsg.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;

    // Publish path
    for (auto &l_node: p_path) {
        // World coordinates for the path
        geometry_msgs::PoseStamped l_poseStamped;
        l_poseStamped.header = l_pathMsg.header;
        l_poseStamped.pose.position.x = l_node.worldCoordinates.x;
        l_poseStamped.pose.position.y = l_node.worldCoordinates.y;
        l_poseStamped.pose.position.z = l_node.worldCoordinates.z;
        l_pathMsg.poses.push_back(l_poseStamped);
    }

    m_targetPathPublisher.publish(l_pathMsg);
}

void Navigation::publishPredictedFootstepSequence(const std::vector<Node> &p_path) {
    int j = 0;
    ROS_INFO_STREAM("Path size: " << p_path.size());

    for (auto &l_node: p_path) {
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

/**
 * Execute planned commands.
 *
 * @param p_path
 */
void Navigation::executePlannedCommands(const std::vector<Node> &p_path) {
    // Full stop command
    sensor_msgs::Joy l_stupidJoy;
    l_stupidJoy.header.stamp = ros::Time::now();
    l_stupidJoy.header.frame_id = "/dev/input/js0";
    l_stupidJoy.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    l_stupidJoy.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Bring robot to full stop once target goal reached
    const double l_stupid = ros::Time::now().toSec();
    while ((ros::Time::now().toSec() - l_stupid) <= 5) {
        // Send motion command
        m_velocityPublisher.publish(l_stupidJoy);

        // Get callback data
        ros::spinOnce();

        // Sleep
        m_rate.sleep();
    }

    // Real path message to be published
    nav_msgs::Path l_realPathMsg;
    l_realPathMsg.header.stamp = ros::Time::now();
    l_realPathMsg.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;

    // Set step height via dynamic reconfigure
    m_drDoubleParam.name = "set_step_height";
    m_drDoubleParam.value = 0.15;
    m_drConf.doubles.push_back(m_drDoubleParam);

    // Send planned command
    for (auto &l_node: p_path) {
        // Set linear velocity via dynamic reconfigure
        m_drDoubleParam.name = "set_linear_vel";
        m_drDoubleParam.value = l_node.velocity;
        m_drConf.doubles.push_back(m_drDoubleParam);

        // Set angular velocity via dynamic reconfigure
        m_drDoubleParam.name = "set_angular_vel";
        m_drDoubleParam.value = l_node.velocity;
        m_drConf.doubles.push_back(m_drDoubleParam);

        // Send ros service call to change dynamic parameters
        m_drSrvReq.config = m_drConf;
        ros::service::call("/aliengo/wb_controller/set_parameters", m_drSrvReq, m_drSrvRes);

        // Time of cache extraction
        ros::Time l_latestPoseTime = ros::Time::now();

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

        // Motion command to send
        sensor_msgs::Joy l_joy;
        l_joy.header.stamp = ros::Time::now();
        l_joy.header.frame_id = "/dev/input/js0";
        l_joy.axes = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        l_joy.buttons = {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};

        ROS_INFO_STREAM("Sending velocity: " << l_node.velocity);
        ROS_INFO_STREAM("Actual FL pose: " << l_latestFLFootPose->pose_actual.position.x << ". Target: " << l_node.feetConfiguration.flCoM.x);
        ROS_INFO_STREAM("Action: " << l_node.action.x << ", " << l_node.action.y << ", " << l_node.action.theta << "\n");

        // Execute command
        while (abs(l_latestFLFootPose->pose_actual.position.x - l_node.feetConfiguration.flCoM.x) >= 0.007) {
            // Send motion command
            m_velocityPublisher.publish(l_joy);


//            m_realPathPublisher.publish(l_realPathMsg);
//            m_targetFeetConfigurationPublisher.publish(m_targetFootsteps[count]);

            // Get callback data
            ros::spinOnce();

            // Sleep
            m_rate.sleep();

            // Acquire new feet positions
            // Time of cache extraction
            l_latestPoseTime = ros::Time::now();

            // Get the latest FL foot pose from the cache
            l_latestFLFootPose = m_flFootPoseCache.getElemBeforeTime(l_latestPoseTime);

            // Get the latest FR foot pose from the cache
            l_latestFRFootPose = m_frFootPoseCache.getElemBeforeTime(l_latestPoseTime);

            // Get the latest RL foot pose from the cache
            l_latestRLFootPose = m_rlFootPoseCache.getElemBeforeTime(l_latestPoseTime);

            // Get the latest RR foot pose from the cache
            l_latestRRFootPose = m_rrFootPoseCache.getElemBeforeTime(l_latestPoseTime);

//            ROS_INFO_STREAM("X distance: " << abs(l_latestFLFootPose->pose_actual.position.x - l_node.feetConfiguration.flCoM.x));
        }
    }

    // Stomp on the spot command
    sensor_msgs::Joy l_stompJoy;
    l_stompJoy.header.stamp = ros::Time::now();
    l_stompJoy.header.frame_id = "/dev/input/js0";
    l_stompJoy.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    l_stompJoy.buttons = {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};

    // Full stop command
    sensor_msgs::Joy l_stopJoy;
    l_stopJoy.header.stamp = ros::Time::now();
    l_stopJoy.header.frame_id = "/dev/input/js0";
    l_stopJoy.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    l_stopJoy.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Stomping behaviour to keep good feet positions
    // once the goal has been reached
    double l_startTime = ros::Time::now().toSec();
    while ((ros::Time::now().toSec() - l_startTime) <= 1.2) {
        // Send motion command
        m_velocityPublisher.publish(l_stompJoy);

        // Get callback data
        ros::spinOnce();

        // Sleep
        m_rate.sleep();
    }

    // Bring robot to full stop once target goal reached
    l_startTime = ros::Time::now().toSec();
    while ((ros::Time::now().toSec() - l_startTime) <= 1.2) {
        // Send motion command
        m_velocityPublisher.publish(l_stopJoy);

        // Get callback data
        ros::spinOnce();

        // Sleep
        m_rate.sleep();
    }
}

/**
 * Plans footsteps location using
 * the most up to date height map
 */
void Navigation::planHeightMapPath(const geometry_msgs::PoseStamped &p_goalMsg) {
    ROS_INFO("Navigation: Planning request received.");

    // Call planner to find path to goal
    std::vector<Node> l_path;
    m_planner.plan(p_goalMsg, l_path);

    // Make sure path is not empty before calling
    // routines that makes use of path information
    if (l_path.empty()) {
        ROS_WARN("Navigation: Path obtained is empty.");
    }

    // Publish CoM path
    publishPredictedCoMPath(l_path);

    // Publish predicted footstep sequence
    publishPredictedFootstepSequence(l_path);

    // Execute planned commands
//    executePlannedCommands(l_path);
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