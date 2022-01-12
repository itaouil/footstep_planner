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
        m_tf2(p_tf2),
        m_rate(1000),
        m_buffer(p_buffer),
        m_planner(p_nh),
        m_swingingFRRL(true),
        m_previousVelocity(0.0),
        m_previousAction{0, 0, 0},
        m_startedJoyPublisher(false) {
    // Wait for time to catch up
    ros::Duration(1).sleep();

    // Path publishers
    m_realPathPublisher = m_nh.advertise<nav_msgs::Path>(REAL_CoM_PATH_TOPIC, 1);
    m_targetPathPublisher = m_nh.advertise<nav_msgs::Path>(PREDICTED_CoM_PATH_TOPIC, 1);

    // Target goal publisher
    m_targetGoalPublisher = m_nh.advertise<geometry_msgs::PoseStamped>(TARGET_GOAL_TOPIC, 1);

    // Velocity command publisher
    m_velocityPublisher = m_nh.advertise<sensor_msgs::Joy>(VELOCITY_CMD_TOPIC, 10);

    // Target goal subscriber
    m_goalSubscriber = m_nh.subscribe("goal", 10, &Navigation::goalCallback, this);

    // Predicted feet configuration marker array publisher
    // Real vs predicted feet configuration marker array publisher
    m_realFeetConfigurationPublisher = m_nh.advertise<visualization_msgs::MarkerArray>(
            REAL_FEET_CONFIGURATION_MARKERS_TOPIC, 1);

    m_targetFeetConfigurationPublisher = m_nh.advertise<visualization_msgs::MarkerArray>(
            PREDICTED_FEET_CONFIGURATION_MARKERS_TOPIC, 1);

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

    // Contact forces subscriber and cache setup
    m_contactForcesSubscriber.subscribe(m_nh, CONTACT_FORCES_TOPIC, 1);
    m_contactForcesCache.connectInput(m_contactForcesSubscriber);
    m_contactForcesCache.setCacheSize(CACHE_SIZE);

    // Acquire initial height map
    if (ACQUIRE_INITIAL_HEIGHT_MAP) buildInitialHeightMap();

    // Spawn thread for joy message publishing
    m_thread = std::thread(&Navigation::joyPublisher, this);
}

/**
 * Destructor
 */
Navigation::~Navigation() {
    // Stop thread
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

/**
 * Stopping behaviour.
 */
void Navigation::halt() {
    sensor_msgs::Joy l_stopJoy;
    l_stopJoy.header.stamp = ros::Time::now();
    l_stopJoy.header.frame_id = "/dev/input/js0";
    l_stopJoy.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    l_stopJoy.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    const double l_startTime = ros::Time::now().toSec();
    while ((ros::Time::now().toSec() - l_startTime) <= 0.5) {
        l_stopJoy.header.stamp = ros::Time::now();
        m_velocityPublisher.publish(l_stopJoy);

        // Sleep
        m_rate.sleep();

        // Get callback data
        ros::spinOnce();
    }
}

/**
 * Stomping behaviour.
 */
void Navigation::stomp() {
    sensor_msgs::Joy l_stompJoy;
    l_stompJoy.header.stamp = ros::Time::now();
    l_stompJoy.header.frame_id = "/dev/input/js0";
    l_stompJoy.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    l_stompJoy.buttons = {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};

    const double l_startTime = ros::Time::now().toSec();
    while ((ros::Time::now().toSec() - l_startTime) <= 1) {
        l_stompJoy.header.stamp = ros::Time::now();
        m_velocityPublisher.publish(l_stompJoy);

        // Sleep
        m_rate.sleep();

        // Get callback data
        ros::spinOnce();
    }
}

/**
 * Reset the robot configuration.
 */
void Navigation::resetConfiguration() {
    // Stomp and stop
    stomp();
    halt();

    // Reset planner variables
    m_swingingFRRL = true;
    m_previousVelocity = 0.0;
    m_previousAction = Action{0, 0, 0};

    ROS_INFO("Navigation: Configuration reset done.");
}

/**
 * Send continuous joy commands
 * to the WBC controller when
 * planned actions are in execution.
 */
void Navigation::joyPublisher() {
    while (ros::ok()) {
        if (m_startedJoyPublisher) {
            m_joy.header.stamp = ros::Time::now();
            m_velocityPublisher.publish(m_joy);

            // Sleep
            m_rate.sleep();

            // Get callback data
            ros::spinOnce();
        }
    }
}

/**
 * Start execution of threaded
 * joy publisher and reset the
 * configuration.
 */
void Navigation::startJoyPublisher() {
    m_joy.header.stamp = ros::Time::now();
    m_joy.header.frame_id = "/dev/input/js0";
    m_joy.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    m_joy.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    resetConfiguration();

    m_startedJoyPublisher = true;
    ROS_INFO("Navigation: Joy publisher started.");
}

/**
 * Stop execution of threaded
 * joy publisher and reset the
 * configuration.
 */
void Navigation::stopJoyPublisher() {
    m_startedJoyPublisher = false;
    resetConfiguration();
    ROS_INFO("Navigation: Joy publisher stopped.");
}

/**
 * Performs a 360 rotation to acquire
 * full height map to be used for planning
 */
void Navigation::buildInitialHeightMap() {
    ROS_INFO("Navigation: Started rotation behaviour to acquire full height map.");

    // Angle of rotation and speed of rotation (in degrees)
    const int l_angle = 360;
    const float l_speed = 10;

    // Convert from angles to radians
    const float l_angularSpeed = l_speed * 2 * M_PI / 360;
    const float l_relativeAngle = l_angle * 2 * M_PI / 360;

    // Initial angle and time
    double l_currentAngle = 0;
    double l_t0 = ros::Time::now().toSec();

    // Angular motion command
    sensor_msgs::Joy l_joy;
    l_joy.header.stamp = ros::Time::now();
    l_joy.header.frame_id = "/dev/input/js0";
    l_joy.axes = {0.0, 0.0, 0.0, -l_angularSpeed, 0.0, 0.0, 0.0, 0.0};
    l_joy.buttons = {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};

    // Send velocity command
    while (l_currentAngle < l_relativeAngle) {
        // Publish message
        m_velocityPublisher.publish(l_joy);

        // Compute current robot direction
        double l_t1 = ros::Time::now().toSec();
        l_currentAngle = l_angularSpeed * (l_t1 - l_t0);

        // Sleep and pull new data
        m_rate.sleep();
        ros::spinOnce();
    }

    // Reset configuration
    resetConfiguration();

    ROS_INFO("Navigation: Rotation behaviour completed.");
}

/**
 * Construct joy command to send.
 *
 * @param p_action
 * @param p_velocity
 */
void Navigation::setJoyCommand(const Action &p_action, const double &p_velocity) {
    // Velocity to be sent
    const auto l_velocityCommand = static_cast<float>(p_velocity);

    // Header setup
    m_joy.header.stamp = ros::Time::now();
    m_joy.header.frame_id = "/dev/input/js0";

    // Set axes and buttons based on action
    if (p_action == Action{1, 0, 0}) {
        m_joy.axes = {0.0, l_velocityCommand, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        m_joy.buttons = {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
    } else if (p_action == Action{0, -1, 0}) {
        m_joy.axes = {-l_velocityCommand, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        m_joy.buttons = {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
    } else if (p_action == Action{0, 1, 0}) {
        m_joy.axes = {l_velocityCommand, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        m_joy.buttons = {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
    } else if (p_action == Action{0, 0, -1}) {
        m_joy.axes = {0.0, 0.0, 0.0, -l_velocityCommand, 0.0, 0.0, 0.0, 0.0};
        m_joy.buttons = {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
    } else if (p_action == Action{0, 0, 1}) {
        m_joy.axes = {0.0, 0.0, 0.0, l_velocityCommand, 0.0, 0.0, 0.0, 0.0};
        m_joy.buttons = {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
    } else {
        ROS_WARN("Navigation: Action is not recognized (joy message)!");
    }
}

/**
 * Plans a path to a target goal
 * using an elevation map (2.5D).
 *
 * @param p_goalMsg
 */
void Navigation::goalCallback(const geometry_msgs::PoseStamped &p_goalMsg) {
    ROS_INFO("Goal callback received");

    // Reset configuration
    resetConfiguration();

    // Save goal for re-planning
    m_goalMsg = p_goalMsg;

    // Plan path
    m_planner.plan(m_goalMsg, m_previousAction, m_previousVelocity, m_swingingFRRL, m_path);

    // Execute planned commands
    executeHighLevelCommands();
}

/**
 * Execute planned commands.
 */
void Navigation::executeHighLevelCommands() {
    while (ros::ok()) {
        unsigned int l_actions = 1;

        if (m_path.empty()) {
            ROS_WARN("Navigation: Path obtained is empty.");
            break;
        }

        // Start joy publisher if not running
        if (!m_startedJoyPublisher) {
            startJoyPublisher();
        }

        // Execute planned commands within horizon
        for (auto &l_node: m_path) {
            ROS_INFO_STREAM("Path size: " << m_path.size());
            ROS_INFO_STREAM("Last executed action: " << m_previousAction.x << ", "
                                                     << m_previousAction.y << ", "
                                                     << m_previousAction.theta);
            ROS_INFO_STREAM("Last executed velocity: " << m_previousVelocity);
            ROS_INFO_STREAM("Last swinging pair: " << m_swingingFRRL << "\n");

            ROS_INFO_STREAM("Count: " << l_actions << "/" << FOOTSTEP_HORIZON);
            ROS_INFO_STREAM("Sending velocity: " << l_node.velocity);
            ROS_INFO_STREAM(
                    "Action: " << l_node.action.x << ", " << l_node.action.y << ", " << l_node.action.theta << "\n");

            // If two different consecutive actions
            // bring robot to idle positions first
            // and then re-plan with the latest
            // configuration reached
//            if (l_node.action != m_previousAction and m_previousAction != Action{0, 0, 0}) {
//                ROS_INFO("Navigation: Different consecutive actions applied. Resetting and re-planning.");
//
//                // Stop joy command and reset
//                stopJoyPublisher();
//
//                // Plan new path
//                m_planner.plan(m_goalMsg, m_previousAction, m_previousVelocity, m_swingingFRRL, m_path);
//
//                break;
//            }

            // Get the latest front feet poses and contact forces
            m_latestFLFootPose = m_flFootPoseCache.getElemBeforeTime(ros::Time::now());
            m_latestFRFootPose = m_frFootPoseCache.getElemBeforeTime(ros::Time::now());
            m_latestContactForces = m_contactForcesCache.getElemBeforeTime(ros::Time::now());

            // Set joy command to be sending
            setJoyCommand(l_node.action, l_node.velocity);

            // Keep sending command until feet contact
            const ros::Time l_startTime = ros::Time::now();
            while ((ros::Time::now().toSec() - l_startTime.toSec()) < 0.15 ||
                   ((m_latestContactForces->contact_forces[0].force.z < 30 &&
                     m_latestContactForces->contact_forces[3].force.z < 30) ||
                    (m_latestContactForces->contact_forces[1].force.z < 30 &&
                     m_latestContactForces->contact_forces[2].force.z < 30))) {
                // Get callback data
                ros::spinOnce();

                // Get latest contact forces data
                m_latestContactForces = m_contactForcesCache.getElemBeforeTime(ros::Time::now());

                ROS_INFO_STREAM("Contacts: " << ros::Time::now().toSec() - l_startTime.toSec() << ", "
                                             << (m_latestContactForces->contact_forces[0].force.z < 30 &&
                                                 m_latestContactForces->contact_forces[3].force.z < 30) << ", "
                                             << (m_latestContactForces->contact_forces[1].force.z < 30 &&
                                                 m_latestContactForces->contact_forces[2].force.z < 30));
            }

            // Add predicted CoM and feet poses
            m_predictedFeetPoses.push_back(l_node);
            m_predictedCoMPoses.push_back(l_node.worldCoordinates);

            // Add actual CoM and feet poses
            const ros::Time l_updateTime = ros::Time::now();
            std::vector<wb_controller::CartesianTask> l_feetConfiguration;
            m_latestRobotPose = m_robotPoseCache.getElemBeforeTime(l_updateTime);
            m_latestFLFootPose = m_flFootPoseCache.getElemBeforeTime(l_updateTime);
            m_latestFRFootPose = m_frFootPoseCache.getElemBeforeTime(l_updateTime);
            m_latestRLFootPose = m_rlFootPoseCache.getElemBeforeTime(l_updateTime);
            m_latestRRFootPose = m_rrFootPoseCache.getElemBeforeTime(l_updateTime);
            l_feetConfiguration.push_back(*m_latestFLFootPose);
            l_feetConfiguration.push_back(*m_latestFRFootPose);
            l_feetConfiguration.push_back(*m_latestRLFootPose);
            l_feetConfiguration.push_back(*m_latestRRFootPose);
            m_realCoMPoses.push_back(*m_latestRobotPose);
            m_realFeetPoses.push_back(l_feetConfiguration);

            // Update planning variables
            m_previousAction = l_node.action;
            m_previousVelocity = l_node.velocity;
            if (m_latestFRFootPose->pose_actual.position.x < m_latestFLFootPose->pose_actual.position.x) {
                ROS_INFO_STREAM("FR and FL x relatively: " << m_latestFRFootPose->pose_actual.position.x << ", "
                                                           << m_latestFLFootPose->pose_actual.position.x);
                m_swingingFRRL = true;
            } else {
                m_swingingFRRL = false;
            }

            ROS_INFO_STREAM("Swinging FR RL: " << m_swingingFRRL);

            // Counter for horizon commands execution
            l_actions += 1;

            // Get callback data
            ros::spinOnce();
        }

        // Plan new path if goal not yet reached
        if ((std::abs(m_latestRobotPose->pose.pose.position.x - m_goalMsg.pose.position.x) > 0.01 ||
             std::abs(m_latestRobotPose->pose.pose.position.y - m_goalMsg.pose.position.y) > 0.01) &&
            m_latestRobotPose->pose.pose.position.x <= m_goalMsg.pose.position.x) {
            m_planner.plan(m_goalMsg, m_previousAction, m_previousVelocity, m_swingingFRRL, m_path);
        } else {
            ROS_INFO("Navigation: Goal has been reached.");
            break;
        }

        // Get callback data
        ros::spinOnce();
    }

    // Stopping joy publisher
    stopJoyPublisher();

    ROS_INFO_STREAM("Publishing predicted and real CoM trajectories");
    publishRealCoMPath();
    publishPredictedCoMPath();

    ROS_INFO_STREAM("Publishing predicted footsteps");
    publishPredictedFootstepSequence();

    ROS_INFO_STREAM("Publishing real footsteps");
    publishRealFootstepSequence();
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
        l_realPoseStamped.pose.position.x = l_pose.pose.pose.position.x;
        l_realPoseStamped.pose.position.y = l_pose.pose.pose.position.y;
        l_realPoseStamped.pose.position.z = l_pose.pose.pose.position.z;
        l_realPathMsg.poses.push_back(l_realPoseStamped);
    }

    m_realPathPublisher.publish(l_realPathMsg);
}

/**
 * Publish predicted CoM path.
 */
void Navigation::publishPredictedCoMPath() {
    if (m_realCoMPoses.empty()) {
        ROS_WARN("Navigation: Predicted CoM path is empty.");
        return;
    }

    nav_msgs::Path l_pathMsg;
    l_pathMsg.header.stamp = ros::Time::now();
    l_pathMsg.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;

    for (auto &l_pose: m_predictedCoMPoses) {
        // World coordinates for the path
        geometry_msgs::PoseStamped l_poseStamped;
        l_poseStamped.header = l_pathMsg.header;
        l_poseStamped.pose.position.x = l_pose.x;
        l_poseStamped.pose.position.y = l_pose.y;
        l_poseStamped.pose.position.z = l_pose.z;
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
void Navigation::publishPredictedFootstepSequence() {
    // Counters
    int j = 0;

    ROS_INFO_STREAM("Size of predicted feet poses: " << m_predictedFeetPoses.size());

    // Populate marker array
    for (auto &l_node: m_predictedFeetPoses) {
        // Feet configuration array
        visualization_msgs::MarkerArray l_targetFeetConfiguration;

        // Populate array
        visualization_msgs::Marker l_footCommonMarker;
        l_footCommonMarker.header.stamp = ros::Time::now();
        l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
        l_footCommonMarker.type = 2;
        l_footCommonMarker.action = 0;
        l_footCommonMarker.lifetime = ros::Duration(1);
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
        ros::Duration(1.5).sleep();
    }
}

/**
 * Publish predicted and real
 * CoM and footstep sequence.
 */
void Navigation::publishRealFootstepSequence() {
    int j = 0;

    if (m_realCoMPoses.empty() || m_realFeetPoses.empty()) {
        ROS_WARN("Navigation: Real feet or CoM containers empty.");
        return;
    }

    for (unsigned int i = 0; i < m_realFeetPoses.size(); i++) {
        // Feet configuration array
        visualization_msgs::MarkerArray l_realFeetConfiguration;

        // Populate target array
        visualization_msgs::Marker l_targetFootCommonMarker;
        l_targetFootCommonMarker.header.stamp = ros::Time::now();
        l_targetFootCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
        l_targetFootCommonMarker.type = 2;
        l_targetFootCommonMarker.action = 0;
        l_targetFootCommonMarker.lifetime = ros::Duration(6);
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
        l_targetCoMMarker.pose.position.x = m_predictedCoMPoses[i].x;
        l_targetCoMMarker.pose.position.y = m_predictedCoMPoses[i].y;
        l_targetCoMMarker.pose.position.z = m_predictedCoMPoses[i].z;

        visualization_msgs::Marker l_targetFLFootMarker = l_targetFootCommonMarker;
        l_targetFLFootMarker.id = j++;
        l_targetFLFootMarker.pose.position.x = m_predictedFeetPoses[i].feetConfiguration.flMap.x;
        l_targetFLFootMarker.pose.position.y = m_predictedFeetPoses[i].feetConfiguration.flMap.y;
        l_targetFLFootMarker.pose.position.z = m_predictedFeetPoses[i].feetConfiguration.flMap.z;

        visualization_msgs::Marker l_targetFRFootMarker = l_targetFootCommonMarker;
        l_targetFRFootMarker.id = j++;
        l_targetFRFootMarker.pose.position.x = m_predictedFeetPoses[i].feetConfiguration.frMap.x;
        l_targetFRFootMarker.pose.position.y = m_predictedFeetPoses[i].feetConfiguration.frMap.y;
        l_targetFRFootMarker.pose.position.z = m_predictedFeetPoses[i].feetConfiguration.frMap.z;

        visualization_msgs::Marker l_targetRLFootMarker = l_targetFootCommonMarker;
        l_targetRLFootMarker.id = j++;
        l_targetRLFootMarker.pose.position.x = m_predictedFeetPoses[i].feetConfiguration.rlMap.x;
        l_targetRLFootMarker.pose.position.y = m_predictedFeetPoses[i].feetConfiguration.rlMap.y;
        l_targetRLFootMarker.pose.position.z = m_predictedFeetPoses[i].feetConfiguration.rlMap.z;

        visualization_msgs::Marker l_targetRRFootMarker = l_targetFootCommonMarker;
        l_targetRRFootMarker.id = j++;
        l_targetRRFootMarker.pose.position.x = m_predictedFeetPoses[i].feetConfiguration.rrMap.x;
        l_targetRRFootMarker.pose.position.y = m_predictedFeetPoses[i].feetConfiguration.rrMap.y;
        l_targetRRFootMarker.pose.position.z = m_predictedFeetPoses[i].feetConfiguration.rrMap.z;

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
        l_realFootCommonMarker.lifetime = ros::Duration(6);
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
        l_realCoMMarker.pose.position.x = m_realCoMPoses[i].pose.pose.position.x;
        l_realCoMMarker.pose.position.y = m_realCoMPoses[i].pose.pose.position.y;
        l_realCoMMarker.pose.position.z = m_realCoMPoses[i].pose.pose.position.z;

        visualization_msgs::Marker l_realFLFootMarker = l_realFootCommonMarker;
        l_realFLFootMarker.id = j++;
        l_realFLFootMarker.pose.position.x =
                m_realCoMPoses[i].pose.pose.position.x + m_realFeetPoses[i][0].pose_actual.position.x;
        l_realFLFootMarker.pose.position.y =
                m_realCoMPoses[i].pose.pose.position.y + m_realFeetPoses[i][0].pose_actual.position.y;
        l_realFLFootMarker.pose.position.z = 0.45 + m_realFeetPoses[i][0].pose_actual.position.z;

        visualization_msgs::Marker l_realFRFootMarker = l_realFootCommonMarker;
        l_realFRFootMarker.id = j++;
        l_realFRFootMarker.pose.position.x =
                m_realCoMPoses[i].pose.pose.position.x + m_realFeetPoses[i][1].pose_actual.position.x;
        l_realFRFootMarker.pose.position.y =
                m_realCoMPoses[i].pose.pose.position.y + m_realFeetPoses[i][1].pose_actual.position.y;
        l_realFRFootMarker.pose.position.z = 0.45 + m_realFeetPoses[i][1].pose_actual.position.z;

        visualization_msgs::Marker l_realRLFootMarker = l_realFootCommonMarker;
        l_realRLFootMarker.id = j++;
        l_realRLFootMarker.pose.position.x =
                m_realCoMPoses[i].pose.pose.position.x + m_realFeetPoses[i][2].pose_actual.position.x;
        l_realRLFootMarker.pose.position.y =
                m_realCoMPoses[i].pose.pose.position.y + m_realFeetPoses[i][2].pose_actual.position.y;
        l_realRLFootMarker.pose.position.z = 0.45 + m_realFeetPoses[i][2].pose_actual.position.z;

        visualization_msgs::Marker l_realRRFootMarker = l_realFootCommonMarker;
        l_realRRFootMarker.id = j++;
        l_realRRFootMarker.pose.position.x =
                m_realCoMPoses[i].pose.pose.position.x + m_realFeetPoses[i][3].pose_actual.position.x;
        l_realRRFootMarker.pose.position.y =
                m_realCoMPoses[i].pose.pose.position.y + m_realFeetPoses[i][3].pose_actual.position.y;
        l_realRRFootMarker.pose.position.z = 0.45 + m_realFeetPoses[i][3].pose_actual.position.z;

        l_realFeetConfiguration.markers.push_back(l_realCoMMarker);
        l_realFeetConfiguration.markers.push_back(l_realFLFootMarker);
        l_realFeetConfiguration.markers.push_back(l_realFRFootMarker);
        l_realFeetConfiguration.markers.push_back(l_realRLFootMarker);
        l_realFeetConfiguration.markers.push_back(l_realRRFootMarker);

        m_realFeetConfigurationPublisher.publish(l_realFeetConfiguration);

        ros::Duration(7).sleep();
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

    // Spin it
    ros::spin();

    return 0;
}