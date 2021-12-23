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
        m_planner(p_nh),
        m_swingingFRRL(false),
        m_currentAction{0, 0, 0},
        m_currentVelocity(0.0) {
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
    // Wait for time to catch up
    ros::Duration(1).sleep();

    // Path publishers
    m_realPathPublisher = m_nh.advertise<nav_msgs::Path>(REAL_CoM_PATH_TOPIC, 1);
    m_targetPathPublisher = m_nh.advertise<nav_msgs::Path>(PREDICTED_CoM_PATH_TOPIC, 1);

    // Velocity command publisher
    m_velocityPublisher = m_nh.advertise<sensor_msgs::Joy>(VELOCITY_CMD_TOPIC, 10);

    // Target goal subscriber
    m_goalSubscriber = m_nh.subscribe("goal", 10, &Navigation::planHeightMapPath, this);

    // Predicted feet configuration marker array publisher
    // Real vs predicted feet configuration marker array publisher
    m_realFeetConfigurationPublisher = m_nh.advertise<visualization_msgs::MarkerArray>(
            REAL_FEET_CONFIGURATION_MARKERS_TOPIC, 1);

    m_targetFeetConfigurationPublisher = m_nh.advertise<visualization_msgs::MarkerArray>(
            PREDICTED_FEET_CONFIGURATION_MARKERS_TOPIC, 1);

    // Robot pose subscriber and cache setup
    m_robotPoseSubscriber.subscribe(m_nh, ODOM_TOPIC, 1);
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

    // Contact forces subscriber and cache setup
    m_contactForcesSubscriber.subscribe(m_nh, CONTACT_FORCES_TOPIC, 1);
    m_contactForcesCache.connectInput(m_contactForcesSubscriber);
    m_contactForcesCache.setCacheSize(CACHE_SIZE);

    // Acquire initial height map
    if (ACQUIRE_INITIAL_HEIGHT_MAP) buildInitialHeightMap();
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

    // Set linear velocity via dynamic reconfigure
    m_drDoubleParam.name = "set_linear_vel";
    m_drDoubleParam.value = 0.0;
    m_drConf.doubles.push_back(m_drDoubleParam);

    // Set angular velocity via dynamic reconfigure
    m_drDoubleParam.name = "set_angular_vel";
    m_drDoubleParam.value = l_angularSpeed;
    m_drConf.doubles.push_back(m_drDoubleParam);

    // Send ros service call to change dynamic parameters
    m_drSrvReq.config = m_drConf;
    ros::service::call("/aliengo/wb_controller/set_parameters", m_drSrvReq, m_drSrvRes);

    // Send ros service call to change dynamic parameters
    m_drSrvReq.config = m_drConf;
    ros::service::call("/aliengo/wb_controller/set_parameters", m_drSrvReq, m_drSrvRes);

    // Setting current variables for distance calculus
    double l_currentAngle = 0;
    double l_t0 = ros::Time::now().toSec();

    // Motion command to send
    sensor_msgs::Joy l_joy;
    l_joy.header.stamp = ros::Time::now();
    l_joy.header.frame_id = "/dev/input/js0";
    l_joy.axes = {-1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0};
    l_joy.buttons = {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};

    // Send velocity command
    while (l_currentAngle < l_relativeAngle) {
        // Publish message
        m_velocityPublisher.publish(l_joy);

        // Compute current robot direction
        double l_t1 = ros::Time::now().toSec();
        l_currentAngle = l_angularSpeed * (l_t1 - l_t0);

        ROS_INFO_STREAM("Time: " << l_t1 << ", " << l_t0);
        ROS_INFO_STREAM(l_currentAngle << ", " << l_relativeAngle);

        m_rate.sleep();

        ros::spinOnce();
    }

    // Set angular velocity via dynamic reconfigure
    m_drDoubleParam.name = "set_angular_vel";
    m_drDoubleParam.value = 0.0;
    m_drConf.doubles.push_back(m_drDoubleParam);

    // Send ros service call to change dynamic parameters
    m_drSrvReq.config = m_drConf;
    ros::service::call("/aliengo/wb_controller/set_parameters", m_drSrvReq, m_drSrvRes);

    ROS_INFO("Navigation: Rotation behaviour completed.");
}

/**
 * Publish real CoM path.
 *
 * @param p_odometry
 */
void Navigation::publishRealCoMPath() {
    if (m_realCoMPoses.empty()) {
        ROS_WARN("Navigation: Real CoM path is empty.");
        return;
    }

    nav_msgs::Path l_realPathMsg;
    l_realPathMsg.header.stamp = ros::Time::now();
    l_realPathMsg.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;

    for (auto &l_pose: m_realCoMPoses) {
        // World coordinates for the path
        geometry_msgs::PoseStamped l_realPoseStamped;
        l_realPoseStamped.header = l_realPathMsg.header;
        l_realPoseStamped.pose.position.x = l_pose.position_actual.x;
        l_realPoseStamped.pose.position.y = l_pose.position_actual.y;
        l_realPoseStamped.pose.position.z = l_pose.position_actual.z;
        l_realPathMsg.poses.push_back(l_realPoseStamped);
    }

    m_realPathPublisher.publish(l_realPathMsg);
}

/**
 * Publish predicted CoM path.
 *
 * @param p_path
 */
void Navigation::publishPredictedCoMPath(const std::vector<Node> &p_path) {
    nav_msgs::Path l_pathMsg;
    l_pathMsg.header.stamp = ros::Time::now();
    l_pathMsg.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;

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

/**
 * Publish predicted CoM and
 * footstep sequence.
 *
 * @param p_path
 */
void Navigation::publishPredictedFootstepSequence(const std::vector<Node> &p_path) {
    ROS_INFO_STREAM("Navigation: Size of predicted footstep sequence is " << p_path.size());

    // Counters
    int j = 0;
    unsigned int count = 0;

    // Populate marker array
    for (auto &l_node: p_path) {
        if (count == 0) {
            ROS_INFO_STREAM("Navigation: Skipping starting node.");
            count += 1;
            continue;
        }

        if (count > FOOTSTEP_HORIZON) {
            continue;
        } else {
            // Feet configuration array
            visualization_msgs::MarkerArray l_targetFeetConfiguration;

            // Populate array
            visualization_msgs::Marker l_footCommonMarker;
            l_footCommonMarker.header.stamp = ros::Time::now();
            l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
            l_footCommonMarker.type = 2;
            l_footCommonMarker.action = 0;
            l_footCommonMarker.lifetime = ros::Duration(2);
            l_footCommonMarker.pose.orientation.x = 0;
            l_footCommonMarker.pose.orientation.y = 0;
            l_footCommonMarker.pose.orientation.z = 0;
            l_footCommonMarker.pose.orientation.w = 1;
            l_footCommonMarker.scale.x = 0.035;
            l_footCommonMarker.scale.y = 0.035;
            l_footCommonMarker.scale.z = 0.035;
            l_footCommonMarker.color.r = 0;
            l_footCommonMarker.color.g = 1;
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
            ros::Duration(3).sleep();

            count += 1;
        }
    }

}

/**
 * Publish predicted and real
 * CoM and footstep sequence.
 *
 * @param p_path
 */
void Navigation::publishRealFootstepSequence(const std::vector<Node> &p_path) {
    int j = 0;

    if (m_realCoMPoses.empty() || m_realFeetPoses.empty()) {
        ROS_WARN("Navigation: Real feet or CoM containers empty.");
        return;
    }

    for (unsigned int i = 0; i < p_path.size(); i++) {
        // Feet configuration array
        visualization_msgs::MarkerArray l_realFeetConfiguration;

        // Populate target array
        visualization_msgs::Marker l_targetFootCommonMarker;
        l_targetFootCommonMarker.header.stamp = ros::Time::now();
        l_targetFootCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
        l_targetFootCommonMarker.type = 2;
        l_targetFootCommonMarker.action = 0;
        l_targetFootCommonMarker.lifetime = ros::Duration(3);
        l_targetFootCommonMarker.pose.orientation.x = 0;
        l_targetFootCommonMarker.pose.orientation.y = 0;
        l_targetFootCommonMarker.pose.orientation.z = 0;
        l_targetFootCommonMarker.pose.orientation.w = 1;
        l_targetFootCommonMarker.scale.x = 0.035;
        l_targetFootCommonMarker.scale.y = 0.035;
        l_targetFootCommonMarker.scale.z = 0.035;
        l_targetFootCommonMarker.color.r = 0;
        l_targetFootCommonMarker.color.g = 1;
        l_targetFootCommonMarker.color.b = 0;
        l_targetFootCommonMarker.color.a = 0.7;

        visualization_msgs::Marker l_targetCoMMarker = l_targetFootCommonMarker;
        l_targetCoMMarker.id = j++;
        l_targetCoMMarker.pose.position.x = p_path[i].worldCoordinates.x;
        l_targetCoMMarker.pose.position.y = p_path[i].worldCoordinates.y;
        l_targetCoMMarker.pose.position.z = p_path[i].worldCoordinates.z;

        visualization_msgs::Marker l_targetFLFootMarker = l_targetFootCommonMarker;
        l_targetFLFootMarker.id = j++;
        l_targetFLFootMarker.pose.position.x = p_path[i].feetConfiguration.flMap.x;
        l_targetFLFootMarker.pose.position.y = p_path[i].feetConfiguration.flMap.y;
        l_targetFLFootMarker.pose.position.z = p_path[i].feetConfiguration.flMap.z;

        visualization_msgs::Marker l_targetFRFootMarker = l_targetFootCommonMarker;
        l_targetFRFootMarker.id = j++;
        l_targetFRFootMarker.pose.position.x = p_path[i].feetConfiguration.frMap.x;
        l_targetFRFootMarker.pose.position.y = p_path[i].feetConfiguration.frMap.y;
        l_targetFRFootMarker.pose.position.z = p_path[i].feetConfiguration.frMap.z;

        visualization_msgs::Marker l_targetRLFootMarker = l_targetFootCommonMarker;
        l_targetRLFootMarker.id = j++;
        l_targetRLFootMarker.pose.position.x = p_path[i].feetConfiguration.rlMap.x;
        l_targetRLFootMarker.pose.position.y = p_path[i].feetConfiguration.rlMap.y;
        l_targetRLFootMarker.pose.position.z = p_path[i].feetConfiguration.rlMap.z;

        visualization_msgs::Marker l_targetRRFootMarker = l_targetFootCommonMarker;
        l_targetRRFootMarker.id = j++;
        l_targetRRFootMarker.pose.position.x = p_path[i].feetConfiguration.rrMap.x;
        l_targetRRFootMarker.pose.position.y = p_path[i].feetConfiguration.rrMap.y;
        l_targetRRFootMarker.pose.position.z = p_path[i].feetConfiguration.rrMap.z;

        l_realFeetConfiguration.markers.push_back(l_targetCoMMarker);
        l_realFeetConfiguration.markers.push_back(l_targetFLFootMarker);
        l_realFeetConfiguration.markers.push_back(l_targetFRFootMarker);
        l_realFeetConfiguration.markers.push_back(l_targetRLFootMarker);
        l_realFeetConfiguration.markers.push_back(l_targetRRFootMarker);

        // Populate real array
        visualization_msgs::Marker l_realFootCommonMarker;
        l_realFootCommonMarker.header.stamp = ros::Time::now();
        l_realFootCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
        l_realFootCommonMarker.type = 2;
        l_realFootCommonMarker.action = 0;
        l_realFootCommonMarker.lifetime = ros::Duration(3);
        l_realFootCommonMarker.pose.orientation.x = 0;
        l_realFootCommonMarker.pose.orientation.y = 0;
        l_realFootCommonMarker.pose.orientation.z = 0;
        l_realFootCommonMarker.pose.orientation.w = 1;
        l_realFootCommonMarker.scale.x = 0.035;
        l_realFootCommonMarker.scale.y = 0.035;
        l_realFootCommonMarker.scale.z = 0.035;
        l_realFootCommonMarker.color.r = 1;
        l_realFootCommonMarker.color.g = 0;
        l_realFootCommonMarker.color.b = 0;
        l_realFootCommonMarker.color.a = 0.7;

        visualization_msgs::Marker l_realCoMMarker = l_realFootCommonMarker;
        l_realCoMMarker.id = j++;
        l_realCoMMarker.pose.position.x = m_realCoMPoses[i].position_actual.x;
        l_realCoMMarker.pose.position.y = m_realCoMPoses[i].position_actual.y;
        l_realCoMMarker.pose.position.z = m_realCoMPoses[i].position_actual.z;

        visualization_msgs::Marker l_realFLFootMarker = l_realFootCommonMarker;
        l_realFLFootMarker.id = j++;
        l_realFLFootMarker.pose.position.x =
                m_realCoMPoses[i].position_actual.x + m_realFeetPoses[i][0].pose_actual.position.x;
        l_realFLFootMarker.pose.position.y =
                m_realCoMPoses[i].position_actual.y + m_realFeetPoses[i][0].pose_actual.position.y;
        l_realFLFootMarker.pose.position.z = p_path[i].feetConfiguration.flMap.z;

        visualization_msgs::Marker l_realFRFootMarker = l_realFootCommonMarker;
        l_realFRFootMarker.id = j++;
        l_realFRFootMarker.pose.position.x =
                m_realCoMPoses[i].position_actual.x + m_realFeetPoses[i][1].pose_actual.position.x;
        l_realFRFootMarker.pose.position.y =
                m_realCoMPoses[i].position_actual.y + m_realFeetPoses[i][1].pose_actual.position.y;
        l_realFRFootMarker.pose.position.z = p_path[i].feetConfiguration.frMap.z;

        visualization_msgs::Marker l_realRLFootMarker = l_realFootCommonMarker;
        l_realRLFootMarker.id = j++;
        l_realRLFootMarker.pose.position.x =
                m_realCoMPoses[i].position_actual.x + m_realFeetPoses[i][2].pose_actual.position.x;
        l_realRLFootMarker.pose.position.y =
                m_realCoMPoses[i].position_actual.y + m_realFeetPoses[i][2].pose_actual.position.y;
        l_realRLFootMarker.pose.position.z = p_path[i].feetConfiguration.rlMap.z;

        visualization_msgs::Marker l_realRRFootMarker = l_realFootCommonMarker;
        l_realRRFootMarker.id = j++;
        l_realRRFootMarker.pose.position.x =
                m_realCoMPoses[i].position_actual.x + m_realFeetPoses[i][3].pose_actual.position.x;
        l_realRRFootMarker.pose.position.y =
                m_realCoMPoses[i].position_actual.y + m_realFeetPoses[i][3].pose_actual.position.y;
        l_realRRFootMarker.pose.position.z = p_path[i].feetConfiguration.rrMap.z;

        l_realFeetConfiguration.markers.push_back(l_realCoMMarker);
        l_realFeetConfiguration.markers.push_back(l_realFLFootMarker);
        l_realFeetConfiguration.markers.push_back(l_realFRFootMarker);
        l_realFeetConfiguration.markers.push_back(l_realRLFootMarker);
        l_realFeetConfiguration.markers.push_back(l_realRRFootMarker);

        m_realFeetConfigurationPublisher.publish(l_realFeetConfiguration);

        ros::Duration(4).sleep();
    }
}

/**
 * Execute planned commands.
 *
 * @param p_path
 */
void Navigation::executePlannedCommands(const std::vector<Node> &p_path) {
    // Local path for re-planning
    std::vector<Node> l_path = p_path;

    // Full stop command
    sensor_msgs::Joy l_stupidJoy;
    l_stupidJoy.header.stamp = ros::Time::now();
    l_stupidJoy.header.frame_id = "/dev/input/js0";
    l_stupidJoy.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    l_stupidJoy.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Clear real CoM and feet poses
    m_realCoMPoses.clear();
    m_realFeetPoses.clear();

    // Set step height via dynamic reconfigure
    m_drDoubleParam.name = "set_step_height";
    m_drDoubleParam.value = 0.15;
    m_drConf.doubles.push_back(m_drDoubleParam);

    // Set angular velocity via dynamic reconfigure
    m_drDoubleParam.name = "set_angular_vel";
    m_drDoubleParam.value = 0.0;
    m_drConf.doubles.push_back(m_drDoubleParam);

    // Send planned command
    while (l_path.size() > 1) {
        // Re-planning variables
        bool l_swingingFRRL;
        Action l_lastAction;
        double l_lastVelocity;

        // Counter
        unsigned int count = 0;

        // Command execution logic
        for (auto &l_node: l_path) {

            // Skip starting root node
            if (count == 0) {
                count += 1;
                continue;
            }

            // Only execute planned commands within horizon
            if (count > FOOTSTEP_HORIZON) {
                l_lastAction = l_node.action;
                l_lastVelocity = l_node.velocity;
                l_swingingFRRL = l_node.feetConfiguration.fr_rl_swinging;
                break;
            }

            // Set linear velocity via dynamic reconfigure
            m_drDoubleParam.name = "set_linear_vel";
            m_drDoubleParam.value = l_node.velocity;
            m_drConf.doubles.push_back(m_drDoubleParam);

            // Send ros service call to change dynamic parameters
            m_drSrvReq.config = m_drConf;
            ros::service::call("/aliengo/wb_controller/set_parameters", m_drSrvReq, m_drSrvRes);

            // Time of cache extraction and start time
            ros::Time l_startTime = ros::Time::now();
            ros::Time l_latestPoseTime = l_startTime;

            // Get the latest odometry pose from the cache
            boost::shared_ptr<wb_controller::ComTask const> l_latestRobotPose =
                    m_robotPoseCache.getElemBeforeTime(l_startTime);

            // Get the latest contact forces from the cache
            boost::shared_ptr<wb_controller::ContactForces const> l_latestContactForces =
                    m_contactForcesCache.getElemBeforeTime(l_startTime);

            // Motion command to send
            sensor_msgs::Joy l_joy;
            l_joy.header.stamp = ros::Time::now();
            l_joy.header.frame_id = "/dev/input/js0";
            l_joy.axes = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            l_joy.buttons = {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};

            ROS_INFO_STREAM("Sending velocity: " << l_node.velocity);
            ROS_INFO_STREAM(
                    "Action: " << l_node.action.x << ", " << l_node.action.y << ", " << l_node.action.theta);

             // Execute command
             while (abs(l_latestRobotPose->position_actual.x - l_node.worldCoordinates.x) >= 0.006) {
                 // Send motion command
                 m_velocityPublisher.publish(l_joy);

                 // Sleep
                 m_rate.sleep();

                 // Get callback data
                 ros::spinOnce();

                 // Time of cache extraction
                 l_latestPoseTime = ros::Time::now();

                 // Update header of the message
                 l_joy.header.stamp = l_latestPoseTime;

                 // Get the latest odometry pose from the cache
                 l_latestRobotPose = m_robotPoseCache.getElemBeforeTime(l_latestPoseTime);

                 // Get the latest contact forces from the cache
                 l_latestContactForces = m_contactForcesCache.getElemBeforeTime(l_latestPoseTime);

                 // ROS_INFO_STREAM(
                 //         "X distance: " << count << "/" << l_path.size() << ", "
                 //                        << abs(l_latestRobotPose->position_actual.x - l_node.worldCoordinates.x)
                 //                        << "\n");
             }

            // Time of cache extraction
            l_latestPoseTime = ros::Time::now();

            // Get the latest odometry pose from the cache
            l_latestRobotPose = m_robotPoseCache.getElemBeforeTime(l_latestPoseTime);

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

            // Logging
            ROS_INFO_STREAM(
                    "X distance: " << count << "/" << l_path.size() << ", "
                                    << abs(l_latestRobotPose->position_actual.x - l_node.worldCoordinates.x) << "\n");

            // Counter for horizon commands execution
            count += 1;

            // Add real CoM trajectory
            m_realCoMPoses.push_back(*l_latestRobotPose);

            // Add real feet poses
            std::vector<wb_controller::CartesianTask> l_feetConfiguration;
            l_feetConfiguration.push_back(*l_latestFLFootPose);
            l_feetConfiguration.push_back(*l_latestFRFootPose);
            l_feetConfiguration.push_back(*l_latestRLFootPose);
            l_feetConfiguration.push_back(*l_latestRRFootPose);
            m_realFeetPoses.push_back(l_feetConfiguration);
        }

        ROS_INFO("Navigation: Re-planning new footstep sequence.");

        // Call planner to find path to goal
        auto start = high_resolution_clock::now();
        l_path.clear();
        m_planner.plan(m_goalMsg, l_lastAction, l_lastVelocity, !l_swingingFRRL, l_path);
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);
        ROS_INFO_STREAM("Time taken by the re-planning: " << duration.count() << " milliseconds" << std::endl);

        // Publish predicted CoM path
        //publishPredictedCoMPath(l_path);

        // Publish predicted footstep sequence
        //publishPredictedFootstepSequence(l_path);
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

        // Sleep
        m_rate.sleep();

        // Get callback data
        ros::spinOnce();
    }

    // Bring robot to full stop once target goal reached
    l_startTime = ros::Time::now().toSec();
    while ((ros::Time::now().toSec() - l_startTime) <= 1.2) {
        // Send motion command
        m_velocityPublisher.publish(l_stopJoy);

        // Sleep
        m_rate.sleep();

        // Get callback data
        ros::spinOnce();
    }
}

/**
 * Plans footsteps location using
 * the most up to date height map
 */
void Navigation::planHeightMapPath(const geometry_msgs::PoseStamped &p_goalMsg) {
    ROS_INFO("Navigation: Planning request received.");

    // Save goal for re-planning
    m_goalMsg = p_goalMsg;

    // Call planner to find path to goal
    std::vector<Node> l_path;
    m_planner.plan(p_goalMsg, Action{0, 0, 0}, 0.0, false, l_path);

    // Make sure path is not empty before calling
    // routines that makes use of path information
    if (l_path.empty()) {
        ROS_WARN("Navigation: Path obtained is empty.");
    }

    // Publish predicted CoM path
    publishPredictedCoMPath(l_path);

    // Publish predicted footstep sequence
    publishPredictedFootstepSequence(l_path);

    // Execute planned commands
    executePlannedCommands(l_path);

    // Publish real CoM path
    publishRealCoMPath();

    // Publish predicted and real footstep sequence
    publishRealFootstepSequence(l_path);
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