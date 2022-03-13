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
    m_joy.buttons = {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};

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
 * Update variables from caches.
 */
void Navigation::updateVariablesFromCache() {
    // Update data from callback
    ros::spinOnce();

    m_latestRobotPose = m_robotPoseCache.getElemBeforeTime(m_robotPoseCache.getLatestTime());
    m_latestFLFootPose = m_flFootPoseCache.getElemBeforeTime(m_flFootPoseCache.getLatestTime());
    m_latestFRFootPose = m_frFootPoseCache.getElemBeforeTime(m_frFootPoseCache.getLatestTime());
    m_latestRLFootPose = m_rlFootPoseCache.getElemBeforeTime(m_rlFootPoseCache.getLatestTime());
    m_latestRRFootPose = m_rrFootPoseCache.getElemBeforeTime(m_rrFootPoseCache.getLatestTime());
    m_latestContactForces = m_contactForcesCache.getElemBeforeTime(m_contactForcesCache.getLatestTime());
}

/**
 * Plans a path to a target goal
 * using an elevation map (2.5D).
 *
 * @param p_goalMsg
 */
void Navigation::goalCallback(const geometry_msgs::PoseStamped &p_goalMsg) {
    ROS_INFO("Goal callback received");

    // Open file stream file
    if (!m_fileStream.is_open()) {
        m_fileStream.open(
                "/home/itaouil/workspace/code/thesis_ws/src/footstep_planner/data/planner_results/parkour2/run1.txt");
    }

    // Reset configuration
    resetConfiguration();

    // Save goal for re-planning
    m_goalMsg = p_goalMsg;

    // Update global variables from cache
    updateVariablesFromCache();

    // Save CoM input to planner
    m_predictionInputCoM.push_back(*m_latestRobotPose);

    // Save feet poses input to planner
    std::vector<wolf_controller::CartesianTask> l_feetConfiguration;
    l_feetConfiguration.push_back(*m_latestFLFootPose);
    l_feetConfiguration.push_back(*m_latestFRFootPose);
    l_feetConfiguration.push_back(*m_latestRLFootPose);
    l_feetConfiguration.push_back(*m_latestRRFootPose);
    m_predictionInputFeet.push_back(l_feetConfiguration);

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
        unsigned int l_actionInExecution = 1;

        if (m_path.empty()) {
            ROS_WARN("Navigation: Path obtained is empty.");
        }

        // Start joy publisher if not running
        if (!m_startedJoyPublisher) {
            startJoyPublisher();
        }

        // Execute planned commands within horizon
        for (auto &l_node: m_path) {
            // Only execute first footstep
            if (l_actionInExecution > 1) {
                break;
            }

            ROS_INFO_STREAM("Path size: " << m_path.size());
            ROS_INFO_STREAM("Last executed action: " << m_previousAction.x << ", "
                                                     << m_previousAction.y << ", "
                                                     << m_previousAction.theta);
            ROS_INFO_STREAM("Last executed velocity: " << m_previousVelocity);
            ROS_INFO_STREAM("Last swinging pair: " << m_swingingFRRL << "\n");

            ROS_INFO_STREAM("Count: " << l_actionInExecution << "/" << FOOTSTEP_HORIZON);
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

            // Update global variables from cache
            updateVariablesFromCache();

            // Set joy command to be sending
            setJoyCommand(l_node.action, l_node.velocity);

            // Contact conditions
            bool l_flInContact = false;
            bool l_rlInContact = false;
            bool l_frInContact = false;
            bool l_rrInContact = false;
            unsigned int l_feetInContact = 0;
            bool l_swingingFeetOutOfContact = false;

            // Publish predicted footsteps in the horizon
            publishOnlinePredictedFootsteps(m_path);

            // Velocity command execution
            const ros::Time l_startTime = ros::Time::now();
            while (l_feetInContact < 3) {
                // Update global variables from cache
                updateVariablesFromCache();

                // Update callback data
                ros::spinOnce();

                // Get feet forces
                auto l_flForceZ = m_latestContactForces->contact_forces[0].force.z;
                auto l_rlForceZ = m_latestContactForces->contact_forces[1].force.z;
                auto l_frForceZ = m_latestContactForces->contact_forces[2].force.z;
                auto l_rrForceZ = m_latestContactForces->contact_forces[3].force.z;

                // Check if height peak was reached
                if (!l_swingingFeetOutOfContact &&
                    ((l_flForceZ < 2 && l_rrForceZ < 2) || (l_frForceZ < 2 && l_rlForceZ < 2))) {
                    l_swingingFeetOutOfContact = true;
                    ROS_INFO_STREAM("Navigation: Feet got out of contact: " << l_flForceZ << ", " << l_frForceZ << ", "
                                                                            << l_rlForceZ << ", " << l_rrForceZ);
                }

                // Update contact booleans once height
                // peak has been reached by any of the
                // two swinging feet
                if (l_swingingFeetOutOfContact) {
                    if (!l_flInContact && l_flForceZ >= 30) {
                        l_flInContact = true;
                        l_feetInContact += 1;
                    }
                    if (!l_rlInContact && l_rlForceZ >= 30) {
                        l_rlInContact = true;
                        l_feetInContact += 1;
                    }
                    if (!l_frInContact && l_frForceZ >= 30) {
                        l_frInContact = true;
                        l_feetInContact += 1;
                    }
                    if (!l_rrInContact && l_rrForceZ >= 30) {
                        l_rrInContact = true;
                        l_feetInContact += 1;
                    }
                }
            }

            ROS_INFO_STREAM("Time at e.o.c: " << ros::Time::now().toSec() - l_startTime.toSec());

            // Add commanded velocity
            m_velocities.push_back(l_node.velocity);

            // Add predicted CoM and footsteps
            m_predictedCoMPoses.push_back(l_node.worldCoordinates);
            m_predictedFootsteps.push_back(l_node.feetConfiguration);

            // Update global variables from cache
            updateVariablesFromCache();

            // Get current feet poses
            geometry_msgs::TransformStamped l_flMap;
            geometry_msgs::TransformStamped l_frMap;
            geometry_msgs::TransformStamped l_rlMap;
            geometry_msgs::TransformStamped l_rrMap;
            geometry_msgs::TransformStamped l_comMap;
            try {
                l_flMap = m_buffer.lookupTransform("world", "lf_foot", ros::Time(0));
                l_frMap = m_buffer.lookupTransform("world", "rf_foot", ros::Time(0));
                l_rlMap = m_buffer.lookupTransform("world", "lh_foot", ros::Time(0));
                l_rrMap = m_buffer.lookupTransform("world", "rh_foot", ros::Time(0));
                l_comMap = m_buffer.lookupTransform("world", "trunk", ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("Planner: Could not transform feet poses from CoM frame to map frame.");
                return;
            }

            // Add actual CoM and footsteps
            std::vector<geometry_msgs::TransformStamped> l_feetConfiguration;
            l_feetConfiguration.push_back(l_flMap);
            l_feetConfiguration.push_back(l_frMap);
            l_feetConfiguration.push_back(l_rlMap);
            l_feetConfiguration.push_back(l_rrMap);
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

            // Log data
            // Add to file the predicted and actual poses
            m_fileStream << m_latestRobotPose->pose.pose.position.x << ","
                         << m_latestRobotPose->pose.pose.position.y << ", "
                         << l_node.worldCoordinates.x << ", "
                         << l_node.worldCoordinates.y << ", "
                         << l_flMap.transform.translation.x << ", "
                         << l_flMap.transform.translation.y << ", "
                         << l_frMap.transform.translation.x << ", "
                         << l_frMap.transform.translation.y << ", "
                         << l_rlMap.transform.translation.x << ", "
                         << l_rlMap.transform.translation.y << ", "
                         << l_rrMap.transform.translation.x << ", "
                         << l_rrMap.transform.translation.y << ", "
                         << l_node.feetConfiguration.flMap.x << ", "
                         << l_node.feetConfiguration.flMap.y << ", "
                         << l_node.feetConfiguration.frMap.x << ", "
                         << l_node.feetConfiguration.frMap.y << ", "
                         << l_node.feetConfiguration.rlMap.x << ", "
                         << l_node.feetConfiguration.rlMap.y << ", "
                         << l_node.feetConfiguration.rrMap.x << ", "
                         << l_node.feetConfiguration.rrMap.y << ", "
                         << l_node.velocity << "\n";

            // Counter for horizon commands execution
            l_actionInExecution += 1;

            // Get callback data
            ros::spinOnce();
        }

        // Plan new path if goal not yet reached
        if ((std::abs(m_latestRobotPose->pose.pose.position.x - m_goalMsg.pose.position.x) > 0.01 ||
             std::abs(m_latestRobotPose->pose.pose.position.y - m_goalMsg.pose.position.y) > 0.01) &&
            m_latestRobotPose->pose.pose.position.x <= m_goalMsg.pose.position.x) {
            // Update global variables from cache
            updateVariablesFromCache();

            // Save CoM input to planner
            m_predictionInputCoM.push_back(*m_latestRobotPose);

            // Save feet poses input to planner
            std::vector<wolf_controller::CartesianTask> l_feetConfiguration;
            l_feetConfiguration.push_back(*m_latestFLFootPose);
            l_feetConfiguration.push_back(*m_latestFRFootPose);
            l_feetConfiguration.push_back(*m_latestRLFootPose);
            l_feetConfiguration.push_back(*m_latestRRFootPose);
            m_predictionInputFeet.push_back(l_feetConfiguration);

            // Plan
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

    // Print planner stats
    m_planner.stats();

    // Close file
    m_fileStream.close();
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

    for (uint i = 0; i < m_predictedCoMPoses.size(); i++) {
        // World coordinates for the path
        geometry_msgs::PoseStamped l_poseStamped;
        l_poseStamped.header = l_pathMsg.header;
        l_poseStamped.pose.position.x = m_predictedCoMPoses[i].x;
        l_poseStamped.pose.position.y = m_predictedCoMPoses[i].y;
        l_poseStamped.pose.position.z = m_realCoMPoses[i].pose.pose.position.z;
        l_pathMsg.poses.push_back(l_poseStamped);
    }

    m_targetPathPublisher.publish(l_pathMsg);
}

/**
 * Publish online the
 * predicted footsteps.
 *
 * @param p_path
 */
void Navigation::publishOnlinePredictedFootsteps(const std::vector<Node> &p_path) {
    // Counters
    int j = 0;

    // Populate marker array
    for (unsigned int i = 0; i < FOOTSTEP_HORIZON; i++) {
        visualization_msgs::MarkerArray l_onlineConfiguration;

        ROS_INFO_STREAM(i);

        visualization_msgs::Marker l_predictionCommon;
        l_predictionCommon.header.stamp = ros::Time::now();
        l_predictionCommon.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
        l_predictionCommon.type = 2;
        l_predictionCommon.action = 0;
        l_predictionCommon.lifetime = ros::Duration(0.3);
        l_predictionCommon.pose.orientation.x = 0;
        l_predictionCommon.pose.orientation.y = 0;
        l_predictionCommon.pose.orientation.z = 0;
        l_predictionCommon.pose.orientation.w = 1;
        l_predictionCommon.scale.x = 0.025;
        l_predictionCommon.scale.y = 0.025;
        l_predictionCommon.scale.z = 0.025;

        visualization_msgs::Marker l_predictedFL = l_predictionCommon;
        l_predictedFL.id = j++;
        l_predictedFL.color.r = 1;
        l_predictedFL.color.g = 0.501;
        l_predictedFL.color.b = 0;
        l_predictedFL.color.a = static_cast<float>(1 - (i * 0.1));
        l_predictedFL.pose.position.x = p_path[i].feetConfiguration.flMap.x;
        l_predictedFL.pose.position.y = p_path[i].feetConfiguration.flMap.y;
        l_predictedFL.pose.position.z = p_path[i].feetConfiguration.flMap.z;

        visualization_msgs::Marker l_predictedFR = l_predictionCommon;
        l_predictedFR.id = j++;
        l_predictedFR.color.r = 1;
        l_predictedFR.color.g = 0.4;
        l_predictedFR.color.b = 0.4;
        l_predictedFR.color.a = static_cast<float>(1 - (i * 0.1));
        l_predictedFR.pose.position.x = p_path[i].feetConfiguration.frMap.x;
        l_predictedFR.pose.position.y = p_path[i].feetConfiguration.frMap.y;
        l_predictedFR.pose.position.z = p_path[i].feetConfiguration.frMap.z;

        visualization_msgs::Marker l_predictedRL = l_predictionCommon;
        l_predictedRL.id = j++;
        l_predictedRL.id = j++;
        l_predictedRL.color.r = 0;
        l_predictedRL.color.g = 1;
        l_predictedRL.color.b = 0.501;
        l_predictedRL.color.a = static_cast<float>(1 - (i * 0.1));
        l_predictedRL.pose.position.x = p_path[i].feetConfiguration.rlMap.x;
        l_predictedRL.pose.position.y = p_path[i].feetConfiguration.rlMap.y;
        l_predictedRL.pose.position.z = p_path[i].feetConfiguration.rlMap.z;

        visualization_msgs::Marker l_predictedRR = l_predictionCommon;
        l_predictedRR.id = j++;
        l_predictedRR.id = j++;
        l_predictedRR.color.r = 0;
        l_predictedRR.color.g = 0.501;
        l_predictedRR.color.b = 1;
        l_predictedRR.color.a = static_cast<float>(1 - (i * 0.1));
        l_predictedRR.pose.position.x = p_path[i].feetConfiguration.rrMap.x;
        l_predictedRR.pose.position.y = p_path[i].feetConfiguration.rrMap.y;
        l_predictedRR.pose.position.z = p_path[i].feetConfiguration.rrMap.z;

        l_onlineConfiguration.markers.push_back(l_predictedFL);
        l_onlineConfiguration.markers.push_back(l_predictedFR);
        l_onlineConfiguration.markers.push_back(l_predictedRL);
        l_onlineConfiguration.markers.push_back(l_predictedRR);

        m_targetFeetConfigurationPublisher.publish(l_onlineConfiguration);
    }
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

    // Populate marker array
    for (unsigned int i = 0; i < m_predictedFootsteps.size(); i++) {
        visualization_msgs::MarkerArray l_predictionConfiguration;

        visualization_msgs::Marker l_predictionCommon;
        l_predictionCommon.header.stamp = ros::Time::now();
        l_predictionCommon.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
        l_predictionCommon.type = 2;
        l_predictionCommon.action = 0;
        l_predictionCommon.lifetime = ros::Duration(2);
        l_predictionCommon.pose.orientation.x = 0;
        l_predictionCommon.pose.orientation.y = 0;
        l_predictionCommon.pose.orientation.z = 0;
        l_predictionCommon.pose.orientation.w = 1;
        l_predictionCommon.scale.x = 0.025;
        l_predictionCommon.scale.y = 0.025;
        l_predictionCommon.scale.z = 0.025;
        l_predictionCommon.color.r = 0;
        l_predictionCommon.color.g = 1;
        l_predictionCommon.color.b = 0;
        l_predictionCommon.color.a = 1;

        visualization_msgs::Marker l_predictedCoM = l_predictionCommon;
        l_predictedCoM.id = j++;
        l_predictedCoM.pose.position.x = m_predictedCoMPoses[i].x;
        l_predictedCoM.pose.position.y = m_predictedCoMPoses[i].y;
        l_predictedCoM.pose.position.z = m_predictedCoMPoses[i].z;

        visualization_msgs::Marker l_predictedFL = l_predictionCommon;
        l_predictedFL.id = j++;
        l_predictedFL.pose.position.x = m_predictedFootsteps[i].flMap.x;
        l_predictedFL.pose.position.y = m_predictedFootsteps[i].flMap.y;
        l_predictedFL.pose.position.z = m_predictedFootsteps[i].flMap.z;

        visualization_msgs::Marker l_predictedFR = l_predictionCommon;
        l_predictedFR.id = j++;
        l_predictedFR.pose.position.x = m_predictedFootsteps[i].frMap.x;
        l_predictedFR.pose.position.y = m_predictedFootsteps[i].frMap.y;
        l_predictedFR.pose.position.z = m_predictedFootsteps[i].frMap.z;

        visualization_msgs::Marker l_predictedRL = l_predictionCommon;
        l_predictedRL.id = j++;
        l_predictedRL.pose.position.x = m_predictedFootsteps[i].rlMap.x;
        l_predictedRL.pose.position.y = m_predictedFootsteps[i].rlMap.y;
        l_predictedRL.pose.position.z = m_predictedFootsteps[i].rlMap.z;

        visualization_msgs::Marker l_predictedRR = l_predictionCommon;
        l_predictedRR.id = j++;
        l_predictedRR.pose.position.x = m_predictedFootsteps[i].rrMap.x;
        l_predictedRR.pose.position.y = m_predictedFootsteps[i].rrMap.y;
        l_predictedRR.pose.position.z = m_predictedFootsteps[i].rrMap.z;

        l_predictionConfiguration.markers.push_back(l_predictedCoM);
        l_predictionConfiguration.markers.push_back(l_predictedFL);
        l_predictionConfiguration.markers.push_back(l_predictedFR);
        l_predictionConfiguration.markers.push_back(l_predictedRL);
        l_predictionConfiguration.markers.push_back(l_predictedRR);

//        visualization_msgs::Marker l_inputFootCommon;
//        l_inputFootCommon.header.stamp = ros::Time::now();
//        l_inputFootCommon.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
//        l_inputFootCommon.type = 2;
//        l_inputFootCommon.action = 0;
//        l_inputFootCommon.lifetime = ros::Duration(2);
//        l_inputFootCommon.pose.orientation.x = 0;
//        l_inputFootCommon.pose.orientation.y = 0;
//        l_inputFootCommon.pose.orientation.z = 0;
//        l_inputFootCommon.pose.orientation.w = 1;
//        l_inputFootCommon.scale.x = 0.025;
//        l_inputFootCommon.scale.y = 0.025;
//        l_inputFootCommon.scale.z = 0.025;
//        l_inputFootCommon.color.r = 0;
//        l_inputFootCommon.color.g = 0;
//        l_inputFootCommon.color.b = 1;
//        l_inputFootCommon.color.a = 1;
//
//        visualization_msgs::Marker l_inputCoM = l_inputFootCommon;
//        l_inputCoM.id = j++;
//        l_inputCoM.pose.position.x = m_predictionInputCoM[i].pose.pose.position.x;
//        l_inputCoM.pose.position.y = m_predictionInputCoM[i].pose.pose.position.y;
//        l_inputCoM.pose.position.z = m_predictionInputCoM[i].pose.pose.position.z;
//
//        visualization_msgs::Marker l_inputFL = l_inputFootCommon;
//        l_inputFL.id = j++;
//        l_inputFL.pose.position.x =
//                m_predictionInputCoM[i].pose.pose.position.x + m_predictionInputFeet[i][0].pose_actual.position.x;
//        l_inputFL.pose.position.y =
//                m_predictionInputCoM[i].pose.pose.position.y + m_predictionInputFeet[i][0].pose_actual.position.y;
//        l_inputFL.pose.position.z =
//                m_predictionInputCoM[i].pose.pose.position.z + m_predictionInputFeet[i][0].pose_actual.position.z;
//
//        visualization_msgs::Marker l_inputFR = l_inputFootCommon;
//        l_inputFR.id = j++;
//        l_inputFR.pose.position.x =
//                m_predictionInputCoM[i].pose.pose.position.x + m_predictionInputFeet[i][1].pose_actual.position.x;
//        l_inputFR.pose.position.y =
//                m_predictionInputCoM[i].pose.pose.position.y + m_predictionInputFeet[i][1].pose_actual.position.y;
//        l_inputFR.pose.position.z =
//                m_predictionInputCoM[i].pose.pose.position.z + m_predictionInputFeet[i][1].pose_actual.position.z;
//
//        visualization_msgs::Marker l_inputRL = l_inputFootCommon;
//        l_inputRL.id = j++;
//        l_inputRL.pose.position.x =
//                m_predictionInputCoM[i].pose.pose.position.x + m_predictionInputFeet[i][2].pose_actual.position.x;
//        l_inputRL.pose.position.y =
//                m_predictionInputCoM[i].pose.pose.position.y + m_predictionInputFeet[i][2].pose_actual.position.y;
//        l_inputRL.pose.position.z =
//                m_predictionInputCoM[i].pose.pose.position.z + m_predictionInputFeet[i][2].pose_actual.position.z;
//
//        visualization_msgs::Marker l_inputRR = l_inputFootCommon;
//        l_inputRR.id = j++;
//        l_inputRR.pose.position.x =
//                m_predictionInputCoM[i].pose.pose.position.x + m_predictionInputFeet[i][3].pose_actual.position.x;
//        l_inputRR.pose.position.y =
//                m_predictionInputCoM[i].pose.pose.position.y + m_predictionInputFeet[i][3].pose_actual.position.y;
//        l_inputRR.pose.position.z =
//                m_predictionInputCoM[i].pose.pose.position.z + m_predictionInputFeet[i][3].pose_actual.position.z;
//
//        l_predictionConfiguration.markers.push_back(l_inputCoM);
//        l_predictionConfiguration.markers.push_back(l_inputFL);
//        l_predictionConfiguration.markers.push_back(l_inputFR);
//        l_predictionConfiguration.markers.push_back(l_inputRL);
//        l_predictionConfiguration.markers.push_back(l_inputRR);

        m_targetFeetConfigurationPublisher.publish(l_predictionConfiguration);
        ros::Duration(2.5).sleep();
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
        l_targetFootCommonMarker.lifetime = ros::Duration(1);
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
        l_targetFootCommonMarker.color.a = 0.8;

        visualization_msgs::Marker l_targetCoMMarker = l_targetFootCommonMarker;
        l_targetCoMMarker.id = j++;
        l_targetCoMMarker.pose.position.x = m_predictedCoMPoses[i].x;
        l_targetCoMMarker.pose.position.y = m_predictedCoMPoses[i].y;
        l_targetCoMMarker.pose.position.z = m_realCoMPoses[i].pose.pose.position.z;

        visualization_msgs::Marker l_targetFLFootMarker = l_targetFootCommonMarker;
        l_targetFLFootMarker.id = j++;
        l_targetFLFootMarker.pose.position.x = m_predictedFootsteps[i].flMap.x;
        l_targetFLFootMarker.pose.position.y = m_predictedFootsteps[i].flMap.y;
        l_targetFLFootMarker.pose.position.z = m_predictedFootsteps[i].flMap.z;

        visualization_msgs::Marker l_targetFRFootMarker = l_targetFootCommonMarker;
        l_targetFRFootMarker.id = j++;
        l_targetFRFootMarker.pose.position.x = m_predictedFootsteps[i].frMap.x;
        l_targetFRFootMarker.pose.position.y = m_predictedFootsteps[i].frMap.y;
        l_targetFRFootMarker.pose.position.z = m_predictedFootsteps[i].frMap.z;

        visualization_msgs::Marker l_targetRLFootMarker = l_targetFootCommonMarker;
        l_targetRLFootMarker.id = j++;
        l_targetRLFootMarker.pose.position.x = m_predictedFootsteps[i].rlMap.x;
        l_targetRLFootMarker.pose.position.y = m_predictedFootsteps[i].rlMap.y;
        l_targetRLFootMarker.pose.position.z = m_predictedFootsteps[i].rlMap.z;

        visualization_msgs::Marker l_targetRRFootMarker = l_targetFootCommonMarker;
        l_targetRRFootMarker.id = j++;
        l_targetRRFootMarker.pose.position.x = m_predictedFootsteps[i].rrMap.x;
        l_targetRRFootMarker.pose.position.y = m_predictedFootsteps[i].rrMap.y;
        l_targetRRFootMarker.pose.position.z = m_predictedFootsteps[i].rrMap.z;

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
        l_realFootCommonMarker.lifetime = ros::Duration(1);
        l_realFootCommonMarker.pose.orientation.x = 0;
        l_realFootCommonMarker.pose.orientation.y = 0;
        l_realFootCommonMarker.pose.orientation.z = 0;
        l_realFootCommonMarker.pose.orientation.w = 1;
        l_realFootCommonMarker.scale.x = 0.035;
        l_realFootCommonMarker.scale.y = 0.035;
        l_realFootCommonMarker.scale.z = 0.035;
        l_realFootCommonMarker.color.r = 0;
        l_realFootCommonMarker.color.g = 0;
        l_realFootCommonMarker.color.b = 1;
        l_realFootCommonMarker.color.a = 0.8;

        visualization_msgs::Marker l_realCoMMarker = l_realFootCommonMarker;
        l_realCoMMarker.id = j++;
        l_realCoMMarker.pose.position.x = m_realCoMPoses[i].pose.pose.position.x;
        l_realCoMMarker.pose.position.y = m_realCoMPoses[i].pose.pose.position.y;
        l_realCoMMarker.pose.position.z = m_realCoMPoses[i].pose.pose.position.z;

        visualization_msgs::Marker l_realFLFootMarker = l_realFootCommonMarker;
        l_realFLFootMarker.id = j++;
        l_realFLFootMarker.pose.position.x = m_realFeetPoses[i][0].transform.translation.x;
        l_realFLFootMarker.pose.position.y = m_realFeetPoses[i][0].transform.translation.y;
        l_realFLFootMarker.pose.position.z = m_predictedFootsteps[i].flMap.z;

        visualization_msgs::Marker l_realFRFootMarker = l_realFootCommonMarker;
        l_realFRFootMarker.id = j++;
        l_realFRFootMarker.pose.position.x = m_realFeetPoses[i][1].transform.translation.x;
        l_realFRFootMarker.pose.position.y = m_realFeetPoses[i][1].transform.translation.y;
        l_realFRFootMarker.pose.position.z = m_predictedFootsteps[i].frMap.z;

        visualization_msgs::Marker l_realRLFootMarker = l_realFootCommonMarker;
        l_realRLFootMarker.id = j++;
        l_realRLFootMarker.pose.position.x = m_realFeetPoses[i][2].transform.translation.x;
        l_realRLFootMarker.pose.position.y = m_realFeetPoses[i][2].transform.translation.y;
        l_realRLFootMarker.pose.position.z = m_predictedFootsteps[i].rlMap.z;

        visualization_msgs::Marker l_realRRFootMarker = l_realFootCommonMarker;
        l_realRRFootMarker.id = j++;
        l_realRRFootMarker.pose.position.x = m_realFeetPoses[i][3].transform.translation.x;
        l_realRRFootMarker.pose.position.y = m_realFeetPoses[i][3].transform.translation.y;
        l_realRRFootMarker.pose.position.z = m_predictedFootsteps[i].rrMap.z;

        l_realFeetConfiguration.markers.push_back(l_realCoMMarker);
        l_realFeetConfiguration.markers.push_back(l_realFLFootMarker);
        l_realFeetConfiguration.markers.push_back(l_realFRFootMarker);
        l_realFeetConfiguration.markers.push_back(l_realRLFootMarker);
        l_realFeetConfiguration.markers.push_back(l_realRRFootMarker);

        m_realFeetConfigurationPublisher.publish(l_realFeetConfiguration);

        ros::Duration(1).sleep();
    }
}

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "footstep_planner");
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