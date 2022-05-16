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
        m_swingingFRRL(false),
        m_previousVelocity(0.0),
        m_previousAction{0, 0, 0},
        m_startedCmdPublisher(false) {
    // Wait for time to catch up
    ros::Duration(1).sleep();

    // Path publishers
    m_realPathPublisher = m_nh.advertise<nav_msgs::Path>(REAL_CoM_PATH_TOPIC, 1);
    m_targetPathPublisher = m_nh.advertise<nav_msgs::Path>(PREDICTED_CoM_PATH_TOPIC, 1);

    // Velocity command publisher
    m_velocityPublisher = m_nh.advertise<unitree_legged_msgs::HighCmd>(VELOCITY_CMD_TOPIC, 1);

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

    // High state subscriber and cache setup
    m_highStateSubscriber.subscribe(m_nh, HIGH_STATE_TOPIC, 1);
    m_highStateCache.connectInput(m_highStateSubscriber);
    m_highStateCache.setCacheSize(CACHE_SIZE);

    // Acquire initial height map
    if (ACQUIRE_INITIAL_HEIGHT_MAP) buildInitialHeightMap();

    // Spawn thread for high level cmd publishing
    m_thread = std::thread(&Navigation::cmdPublisher, this);
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
 * Send continuous high level 
 * commands to the controller when
 * planned actions are in execution.
 */
void Navigation::cmdPublisher() {
    while (ros::ok()) {
        if (m_startedCmdPublisher) {
            m_velocityPublisher.publish(m_cmd);

            // Sleep
            m_rate.sleep();

            // Get callback data
            ros::spinOnce();
        }
    }
}

/**
 * Start execution of threaded
 * cmd publisher and reset the
 * configuration.
 */
void Navigation::startCmdPublisher() {
    m_cmd.mode = 2;
    m_cmd.gaitType = 0;
    m_cmd.speedLevel = 2;
    m_cmd.dFootRaiseHeight = 0.0f;
    m_cmd.dBodyHeight = 0.0f;
    m_cmd.position[0] = 0.0f;
    m_cmd.position[1] = 0.0f;
    m_cmd.rpy[0] = 0.0f;
    m_cmd.rpy[1] = 0.0f;
    m_cmd.rpy[2] = 0.0f;

    m_startedCmdPublisher = true;
    ROS_INFO("Navigation: Cmd publisher started.");
}

/**
 * Stop execution of threaded
 * cmd publisher and reset the
 * configuration.
 */
void Navigation::stopCmdPublisher() {
    m_startedCmdPublisher = false;
    ROS_INFO("Navigation: Cmd publisher stopped.");
}

/**
 * Performs a 360 rotation to acquire
 * full height map to be used for planning
 */
void Navigation::buildInitialHeightMap() {
    ROS_INFO("Navigation: Started rotation behaviour to acquire full height map.");

    // Angle of rotation and speed of rotation (in degrees)
    const int l_angle = 360;
    const float l_speed = 5;

    // Convert from angles to radians
    const float l_angularSpeed = l_speed * 2 * M_PI / 360;
    const float l_relativeAngle = l_angle * 2 * M_PI / 360;

    // Initial angle and time
    double l_currentAngle = 0;
    double l_t0 = ros::Time::now().toSec();

    // Angular motion high level command
    unitree_legged_msgs::HighCmd l_cmd;
    l_cmd.mode = 2;
    l_cmd.gaitType = 0;
    l_cmd.speedLevel = 2;
    l_cmd.dFootRaiseHeight = 0.0f;
    l_cmd.dBodyHeight = 0.0f;
    l_cmd.position[0] = 0.0f;
    l_cmd.position[1] = 0.0f;
    l_cmd.rpy[0] = 0.0f;
    l_cmd.rpy[1] = 0.0f;
    l_cmd.rpy[2] = 0.0f;
    l_cmd.velocity[0] = 0.0f;
    l_cmd.velocity[1] = 0.0f;
    l_cmd.yawSpeed = l_angularSpeed;

    // Send velocity command
    while (l_currentAngle < l_relativeAngle) {
        // Publish message
        m_velocityPublisher.publish(l_cmd);

        // Compute current robot direction
        double l_t1 = ros::Time::now().toSec();
        l_currentAngle = l_angularSpeed * (l_t1 - l_t0);

        // Sleep and pull new data
        m_rate.sleep();
        ros::spinOnce();
    }

    ROS_INFO("Navigation: Rotation behaviour completed.");
}

/**
 * Construct high level cmd to send.
 *
 * @param p_action
 * @param p_velocity
 */
void Navigation::setCmd(const Action &p_action, const double &p_velocity) {
    // Velocity to be sent
    const auto l_velocityCommand = static_cast<float>(p_velocity);

    m_cmd.mode = 2;
    m_cmd.gaitType = 0;
    m_cmd.speedLevel = 2;
    m_cmd.dFootRaiseHeight = 0.0f;
    m_cmd.dBodyHeight = 0.0f;
    m_cmd.position[0] = 0.0f;
    m_cmd.position[1] = 0.0f;
    m_cmd.rpy[0] = 0.0f;
    m_cmd.rpy[1] = 0.0f;
    m_cmd.rpy[2] = 0.0f;

    // Set axes and buttons based on action
    if (p_action == Action{1, 0, 0}) {
        m_cmd.velocity[0] = p_velocity;
        m_cmd.velocity[1] = 0.0f;
        m_cmd.yawSpeed = 0.0f;
    } else if (p_action == Action{0, -1, 0}) {
        m_cmd.velocity[0] = 0.0f;
        m_cmd.velocity[1] = -p_velocity;
        m_cmd.yawSpeed = 0.0f;
    } else if (p_action == Action{0, 1, 0}) {
        m_cmd.velocity[0] = 0.0f;
        m_cmd.velocity[1] = p_velocity;
        m_cmd.yawSpeed = 0.0f;
    } else if (p_action == Action{0, 0, -1}) {
        m_cmd.velocity[0] = 0.0f;
        m_cmd.velocity[1] = 0.0f;
        m_cmd.yawSpeed = -p_velocity;
    } else if (p_action == Action{0, 0, 1}) {
        m_cmd.velocity[0] = 0.0f;
        m_cmd.velocity[1] = 0.0f;
        m_cmd.yawSpeed = p_velocity;
    } else {
        ROS_WARN("Navigation: Action is not recognized (cmd message)!");
    }
}

/**
 * Update variables from caches.
 */
void Navigation::updateVariablesFromCache() {
    // Update data from callback
    ros::spinOnce();

    boost::shared_ptr<nav_msgs::Odometry const> l_latestT265Pose = m_robotPoseCache.getElemBeforeTime(ros::Time::now());

    // Offset t265 pose to obtain CoM pose
    m_latestCoMPose = *l_latestT265Pose;
    m_latestCoMPose.pose.pose.position.x -= 0.33118;
    m_latestCoMPose.pose.pose.position.z += 0.0045;

    m_latestHighState = m_highStateCache.getElemBeforeTime(ros::Time::now());
    m_latestFLFootPose = m_latestHighState->footPosition2Body[1];
    m_latestFRFootPose = m_latestHighState->footPosition2Body[0];
    m_latestRLFootPose = m_latestHighState->footPosition2Body[3];
    m_latestRRFootPose = m_latestHighState->footPosition2Body[2];
    m_latestContactForces = {m_latestHighState->footForce[1], 
                             m_latestHighState->footForce[0],
                             m_latestHighState->footForce[3],
                             m_latestHighState->footForce[2]};
}

/**
 * Plans a path to a target goal
 * using an elevation map (2.5D).
 *
 * @param p_goalMsg
 */
void Navigation::goalCallback(const geometry_msgs::PoseStamped &p_goalMsg) {
    ROS_INFO("Goal callback received");

    // // Open file stream file
    // if (!m_fileStream.is_open()) {
    //     m_fileStream.open(
    //             "/home/itaouil/workspace/code/thesis_ws/src/footstep_planner/data/planner_results/parkour3/presentation_run1.txt");
    // }

    // Save goal for re-planning
    m_goalMsg = p_goalMsg;

    // Update global variables from cache
    updateVariablesFromCache();

    // Save CoM input to planner
    m_predictionInputCoM.push_back(m_latestCoMPose);

    // Save feet poses input to planner
    std::vector<unitree_legged_msgs::Cartesian> l_feetConfiguration;
    l_feetConfiguration.push_back(m_latestFLFootPose);
    l_feetConfiguration.push_back(m_latestFRFootPose);
    l_feetConfiguration.push_back(m_latestRLFootPose);
    l_feetConfiguration.push_back(m_latestRRFootPose);
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

        // Start cmd publisher if not running
        if (!m_startedCmdPublisher) {
            startCmdPublisher();
        }

        // Execute planned commands within horizon
        for (auto &l_node: m_path) {
            // Only execute first footstep
            if (l_actionInExecution > 1) {
                break;
            }

//            // If two different consecutive actions
//            // bring robot to idle positions first
//            // and then re-plan with the latest
//            // configuration reached
//            if (l_node.action != m_previousAction and m_previousAction != Action{0, 0, 0}) {
//                ROS_INFO("Navigation: Different consecutive actions applied. Resetting and re-planning.");
//
//                // Stop cmd publishing and reset
//                stopCmdPublisher();
//
//                // Plan new path
//                m_planner.plan(m_goalMsg, m_previousAction, m_previousVelocity, m_swingingFRRL, m_path);
//
//                break;
//            }

            // Update global variables from cache
            updateVariablesFromCache();
            
            // Set cmd to be sent
            setCmd(l_node.action, l_node.velocity);

            // Contact conditions
            bool l_flInContact = false;
            bool l_rlInContact = false;
            bool l_frInContact = false;
            bool l_rrInContact = false;
            unsigned int l_feetInContact = 0;
            bool l_swingingFeetOutOfContact = false;

            // Publish predicted footsteps in the horizon
            // publishOnlinePredictedFootsteps(m_path);

            // Velocity command execution
            const ros::Time l_startTime = ros::Time::now();
            while (l_feetInContact < 3) {
                // Update global variables from cache
                updateVariablesFromCache();

                // Update callback data
                ros::spinOnce();

                // Get feet forces
                auto l_flForceZ = m_latestContactForces[0];
                auto l_frForceZ = m_latestContactForces[1];
                auto l_rrForceZ = m_latestContactForces[2];
                auto l_rlForceZ = m_latestContactForces[3];

                // Check if height peak was reached
                if (!l_swingingFeetOutOfContact &&
                    ((l_flForceZ < 10 && l_rrForceZ < 10) || (l_frForceZ < 10 && l_rlForceZ < 10))) {
                    l_swingingFeetOutOfContact = true;
                    ROS_INFO_STREAM("Navigation: Feet got out of contact: " << l_flForceZ << ", " << l_frForceZ << ", "
                                                                            << l_rlForceZ << ", " << l_rrForceZ);
                }

                // Update contact booleans once height
                // peak has been reached by any of the
                // two swinging feet
                if (l_swingingFeetOutOfContact) {
                    if (!l_flInContact && l_flForceZ >= 60) {
                        l_flInContact = true;
                        l_feetInContact += 1;
                    }
                    if (!l_rlInContact && l_rlForceZ >= 60) {
                        l_rlInContact = true;
                        l_feetInContact += 1;
                    }
                    if (!l_frInContact && l_frForceZ >= 60) {
                        l_frInContact = true;
                        l_feetInContact += 1;
                    }
                    if (!l_rrInContact && l_rrForceZ >= 60) {
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
                l_flMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME, "FL_foot", ros::Time(0));
                l_frMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME, "FR_foot", ros::Time(0));
                l_rlMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME, "RL_foot", ros::Time(0));
                l_rrMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME, "RR_foot", ros::Time(0));
                l_comMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME, "base", ros::Time(0));
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
            m_realCoMPoses.push_back(m_latestCoMPose);
            m_realFeetPoses.push_back(l_feetConfiguration);

            // Update planning variables
            m_previousAction = l_node.action;
            m_previousVelocity = l_node.velocity;
            if (m_latestFRFootPose.x < m_latestFLFootPose.x) {
                ROS_INFO_STREAM("FR and FL x relatively: " << m_latestFRFootPose.x << ", "
                                                           << m_latestFLFootPose.x);
                m_swingingFRRL = true;
            } else {
                m_swingingFRRL = false;
            }

            // // Add to file the predicted and actual poses
            // m_fileStream << m_latestCoMPose.pose.pose.position.x << ","
            //              << m_latestCoMPose.pose.pose.position.y << ", "
            //              << l_node.worldCoordinates.x << ", "
            //              << l_node.worldCoordinates.y << ", "
            //              << l_flMap.transform.translation.x << ", "
            //              << l_flMap.transform.translation.y << ", "
            //              << l_frMap.transform.translation.x << ", "
            //              << l_frMap.transform.translation.y << ", "
            //              << l_rlMap.transform.translation.x << ", "
            //              << l_rlMap.transform.translation.y << ", "
            //              << l_rrMap.transform.translation.x << ", "
            //              << l_rrMap.transform.translation.y << ", "
            //              << l_node.feetConfiguration.flMap.x << ", "
            //              << l_node.feetConfiguration.flMap.y << ", "
            //              << l_node.feetConfiguration.frMap.x << ", "
            //              << l_node.feetConfiguration.frMap.y << ", "
            //              << l_node.feetConfiguration.rlMap.x << ", "
            //              << l_node.feetConfiguration.rlMap.y << ", "
            //              << l_node.feetConfiguration.rrMap.x << ", "
            //              << l_node.feetConfiguration.rrMap.y << ", "
            //              << l_node.velocity << "\n";
            // m_fileStream.flush();

            // Counter for horizon commands execution
            l_actionInExecution += 1;

            // Get callback data
            ros::spinOnce();
        }

        // Plan new path if goal not yet reached
        if ((std::abs(m_latestCoMPose.pose.pose.position.x - m_goalMsg.pose.position.x) > 0.01 ||
             std::abs(m_latestCoMPose.pose.pose.position.y - m_goalMsg.pose.position.y) > 0.01) &&
            m_latestCoMPose.pose.pose.position.x <= m_goalMsg.pose.position.x) {
            // Update global variables from cache
            updateVariablesFromCache();

            // Save CoM input to planner
            m_predictionInputCoM.push_back(m_latestCoMPose);

            // Save feet poses input to planner
            // std::vector<unitree_legged_msgs::Cartesian> l_feetConfiguration;
            // l_feetConfiguration.push_back(m_latestFLFootPose);
            // l_feetConfiguration.push_back(m_latestFRFootPose);
            // l_feetConfiguration.push_back(m_latestRLFootPose);
            // l_feetConfiguration.push_back(m_latestRRFootPose);

            // Get current feet poses
            geometry_msgs::TransformStamped l_flMap;
            geometry_msgs::TransformStamped l_frMap;
            geometry_msgs::TransformStamped l_rlMap;
            geometry_msgs::TransformStamped l_rrMap;
            geometry_msgs::TransformStamped l_comMap;
            try {
                l_flMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME, "FL_foot", ros::Time(0));
                l_frMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME, "FR_foot", ros::Time(0));
                l_rlMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME, "RL_foot", ros::Time(0));
                l_rrMap = m_buffer.lookupTransform(HEIGHT_MAP_REFERENCE_FRAME, "RR_foot", ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("Planner: Could not transform feet poses to map frame.");
                return;
            }

            // Transform stamped to cartesian foot
            unitree_legged_msgs::Cartesian l_fl;
            l_fl.x = l_flMap.transform.translation.x;
            l_fl.y = l_flMap.transform.translation.y;
            l_fl.z = l_flMap.transform.translation.z;

            unitree_legged_msgs::Cartesian l_fr;
            l_fr.x = l_frMap.transform.translation.x;
            l_fr.y = l_frMap.transform.translation.y;
            l_fr.z = l_frMap.transform.translation.z;

            unitree_legged_msgs::Cartesian l_rl;
            l_rl.x = l_rlMap.transform.translation.x;
            l_rl.y = l_rlMap.transform.translation.y;
            l_rl.z = l_rlMap.transform.translation.z;

            unitree_legged_msgs::Cartesian l_rr;
            l_rr.x = l_rrMap.transform.translation.x;
            l_rr.y = l_rrMap.transform.translation.y;
            l_rr.z = l_rrMap.transform.translation.z;

            // Add actual CoM and footsteps
            std::vector<unitree_legged_msgs::Cartesian> l_feetConfiguration;
            l_feetConfiguration.push_back(l_fl);
            l_feetConfiguration.push_back(l_fr);
            l_feetConfiguration.push_back(l_rl);
            l_feetConfiguration.push_back(l_rr);
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
    
    // Stop cmd publisher
    stopCmdPublisher();

    ROS_INFO_STREAM("Publishing predicted and real CoM trajectories");
    publishRealCoMPath();
    publishPredictedCoMPath();

    ROS_INFO_STREAM("Publishing predicted footsteps");
    publishPredictedFootstepSequence();

    ROS_INFO_STREAM("Publishing real footsteps");
    publishRealFootstepSequence();

    // Print planner stats
    m_planner.stats();

//    // Close file
//    m_fileStream.close();
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
void Navigation::publishOnlinePredictedFootsteps(std::vector<Node> &p_path) {
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
        l_predictionCommon.scale.x = 0.035;
        l_predictionCommon.scale.y = 0.035;
        l_predictionCommon.scale.z = 0.035;

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
    for (unsigned int i = 1; i < m_predictedFootsteps.size(); i++) {
        visualization_msgs::MarkerArray l_predictionConfiguration;

        visualization_msgs::Marker l_predictionCommon;
        l_predictionCommon.header.stamp = ros::Time::now();
        l_predictionCommon.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
        l_predictionCommon.type = 2;
        l_predictionCommon.action = 0;
        l_predictionCommon.lifetime = ros::Duration(3);
        l_predictionCommon.pose.orientation.x = 0;
        l_predictionCommon.pose.orientation.y = 0;
        l_predictionCommon.pose.orientation.z = 0;
        l_predictionCommon.pose.orientation.w = 1;
        l_predictionCommon.scale.x = 0.04;
        l_predictionCommon.scale.y = 0.04;
        l_predictionCommon.scale.z = 0.04;
        l_predictionCommon.color.r = 0;
        l_predictionCommon.color.g = 1;
        l_predictionCommon.color.b = 0;
        l_predictionCommon.color.a = 0.5;

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

        visualization_msgs::Marker l_inputFootCommon;
        l_inputFootCommon.header.stamp = ros::Time::now();
        l_inputFootCommon.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
        l_inputFootCommon.type = 2;
        l_inputFootCommon.action = 0;
        l_inputFootCommon.lifetime = ros::Duration(3);
        l_inputFootCommon.pose.orientation.x = 0;
        l_inputFootCommon.pose.orientation.y = 0;
        l_inputFootCommon.pose.orientation.z = 0;
        l_inputFootCommon.pose.orientation.w = 1;
        l_inputFootCommon.scale.x = 0.04;
        l_inputFootCommon.scale.y = 0.04;
        l_inputFootCommon.scale.z = 0.04;
        l_inputFootCommon.color.r = 0;
        l_inputFootCommon.color.g = 0;
        l_inputFootCommon.color.b = 1;
        l_inputFootCommon.color.a = 0.5;

        visualization_msgs::Marker l_inputCoM = l_inputFootCommon;
        l_inputCoM.id = j++;
        l_inputCoM.pose.position.x = m_predictionInputCoM[i].pose.pose.position.x;
        l_inputCoM.pose.position.y = m_predictionInputCoM[i].pose.pose.position.y;
        l_inputCoM.pose.position.z = m_predictionInputCoM[i].pose.pose.position.z;

        visualization_msgs::Marker l_inputFL = l_inputFootCommon;
        l_inputFL.id = j++;
        l_inputFL.pose.position.x = m_predictionInputFeet[i][0].x;
        l_inputFL.pose.position.y = m_predictionInputFeet[i][0].y;
        l_inputFL.pose.position.z = m_predictionInputFeet[i][0].z;

        visualization_msgs::Marker l_inputFR = l_inputFootCommon;
        l_inputFR.id = j++;
        l_inputFR.pose.position.x = m_predictionInputFeet[i][1].x;
        l_inputFR.pose.position.y = m_predictionInputFeet[i][1].y;
        l_inputFR.pose.position.z = m_predictionInputFeet[i][1].z;

        visualization_msgs::Marker l_inputRL = l_inputFootCommon;
        l_inputRL.id = j++;
        l_inputRL.pose.position.x = m_predictionInputFeet[i][2].x;
        l_inputRL.pose.position.y = m_predictionInputFeet[i][2].y;
        l_inputRL.pose.position.z = m_predictionInputFeet[i][2].z;

        visualization_msgs::Marker l_inputRR = l_inputFootCommon;
        l_inputRR.id = j++;
        l_inputRR.pose.position.x = m_predictionInputFeet[i][3].x;
        l_inputRR.pose.position.y = m_predictionInputFeet[i][3].y;
        l_inputRR.pose.position.z = m_predictionInputFeet[i][3].z;

        l_predictionConfiguration.markers.push_back(l_inputCoM);
        l_predictionConfiguration.markers.push_back(l_inputFL);
        l_predictionConfiguration.markers.push_back(l_inputFR);
        l_predictionConfiguration.markers.push_back(l_inputRL);
        l_predictionConfiguration.markers.push_back(l_inputRR);

        m_targetFeetConfigurationPublisher.publish(l_predictionConfiguration);
        ros::Duration(3.2).sleep();
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
        l_targetFootCommonMarker.lifetime = ros::Duration(3);
        l_targetFootCommonMarker.pose.orientation.x = 0;
        l_targetFootCommonMarker.pose.orientation.y = 0;
        l_targetFootCommonMarker.pose.orientation.z = 0;
        l_targetFootCommonMarker.pose.orientation.w = 1;
        l_targetFootCommonMarker.scale.x = 0.04;
        l_targetFootCommonMarker.scale.y = 0.04;
        l_targetFootCommonMarker.scale.z = 0.04;
        l_targetFootCommonMarker.color.r = 0;
        l_targetFootCommonMarker.color.g = 1;
        l_targetFootCommonMarker.color.b = 0;
        l_targetFootCommonMarker.color.a = 0.5;

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

//        l_realFeetConfiguration.markers.push_back(l_targetCoMMarker);
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
        l_realFootCommonMarker.scale.x = 0.04;
        l_realFootCommonMarker.scale.y = 0.04;
        l_realFootCommonMarker.scale.z = 0.04;
        l_realFootCommonMarker.color.r = 0;
        l_realFootCommonMarker.color.g = 0;
        l_realFootCommonMarker.color.b = 1;
        l_realFootCommonMarker.color.a = 0.5;

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

//        l_realFeetConfiguration.markers.push_back(l_realCoMMarker);
        l_realFeetConfiguration.markers.push_back(l_realFLFootMarker);
        l_realFeetConfiguration.markers.push_back(l_realFRFootMarker);
        l_realFeetConfiguration.markers.push_back(l_realRLFootMarker);
        l_realFeetConfiguration.markers.push_back(l_realRRFootMarker);

        m_realFeetConfigurationPublisher.publish(l_realFeetConfiguration);
        ros::Duration(3.2).sleep();
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