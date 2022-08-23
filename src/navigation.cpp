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
        m_startedCmdPublisher(false),
        m_previousAction{0, 0, 0} {
    m_realPathPublisher = m_nh.advertise<nav_msgs::Path>(REAL_CoM_PATH_TOPIC, 1);
    m_velocityPublisher = m_nh.advertise<geometry_msgs::TwistStamped>(VELOCITY_CMD_TOPIC, 1);
    m_targetPathPublisher = m_nh.advertise<nav_msgs::Path>(PREDICTED_CoM_PATH_TOPIC, 1);
    m_realFeetConfigurationPublisher = m_nh.advertise<visualization_msgs::MarkerArray>(
        REAL_FEET_CONFIGURATION_MARKERS_TOPIC, 1);
    m_targetFeetConfigurationPublisher = m_nh.advertise<visualization_msgs::MarkerArray>(
            PREDICTED_FEET_CONFIGURATION_MARKERS_TOPIC, 1);

    m_goalSubscriber = m_nh.subscribe("/move_base_simple/goal", 10, &Navigation::goalCallback, this);

    m_robotPoseSubscriber.subscribe(m_nh, ROBOT_POSE_TOPIC, 1);
    m_robotPoseCache.connectInput(m_robotPoseSubscriber);
    m_robotPoseCache.setCacheSize(CACHE_SIZE);

    m_feetForcesSubscriber.subscribe(m_nh, FEET_FORCES_TOPIC, 1);
    m_feetForcesCache.connectInput(m_feetForcesSubscriber);
    m_feetForcesCache.setCacheSize(CACHE_SIZE);

    if (ACQUIRE_INITIAL_HEIGHT_MAP) buildInitialHeightMap();

    m_cmdPubThread = std::thread(&Navigation::cmdPublisher, this);
    m_predFeetThread = std::thread(&Navigation::publishOnlinePredictedFootsteps, this);
}

/**
 * Destructor
 */
Navigation::~Navigation() {
    if (m_cmdPubThread.joinable()) {
        m_cmdPubThread.join();
    }

    if (m_predFeetThread.joinable()) {
        m_predFeetThread.join();
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
            m_velocityPublisher.publish(m_velCmd);
            m_rate.sleep();
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
    m_velCmd.header.stamp = ros::Time::now();
    m_velCmd.twist.linear.x = 0;
    m_velCmd.twist.linear.y = 0;
    m_velCmd.twist.linear.z = 0;
    m_velCmd.twist.angular.x = 0;
    m_velCmd.twist.angular.y = 0;
    m_velCmd.twist.angular.z = 0;
    m_startedCmdPublisher = true;
    ROS_INFO("Navigation: Cmd publisher started.");
}

/**
 * Stop execution of threaded
 * cmd publisher and reset the
 * configuration.
 */
void Navigation::stopCmdPublisher() {
    m_velCmd.header.stamp = ros::Time::now();
    m_velCmd.twist.linear.x = 0;
    m_velCmd.twist.linear.y = 0;
    m_velCmd.twist.linear.z = 0;
    m_velCmd.twist.angular.x = 0;
    m_velCmd.twist.angular.y = 0;
    m_velCmd.twist.angular.z = 0;
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

    // Angular velocity command
    m_velCmd.header.stamp = ros::Time::now();
    m_velCmd.twist.linear.x = 0;
    m_velCmd.twist.linear.y = 0;
    m_velCmd.twist.linear.z = 0;
    m_velCmd.twist.angular.x = 0;
    m_velCmd.twist.angular.y = 0;
    m_velCmd.twist.angular.z = l_angularSpeed;

    // Send velocity command
    while (l_currentAngle < l_relativeAngle) {
        // Publish message
        m_velocityPublisher.publish(m_velCmd);

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

    m_velCmd.header.stamp = ros::Time::now();
    m_velCmd.twist.linear.x = 0;
    m_velCmd.twist.linear.y = 0;
    m_velCmd.twist.linear.z = 0;
    m_velCmd.twist.angular.x = 0;
    m_velCmd.twist.angular.y = 0;
    m_velCmd.twist.angular.z = 0;

    // Set axes and buttons based on action
    if (p_action == Action{1, 0, 0}) {
        m_velCmd.twist.linear.x = p_velocity;
    } else if (p_action == Action{0, -1, 0}) {
        m_velCmd.twist.linear.y = -p_velocity;
    } else if (p_action == Action{0, 1, 0}) {
        m_velCmd.twist.linear.y = p_velocity;
    } else if (p_action == Action{0, 0, -1}) {
        m_velCmd.twist.angular.z = -p_velocity;
    } else if (p_action == Action{0, 0, 1}) {
        m_velCmd.twist.angular.z = p_velocity;
    } else {
        ROS_WARN("Navigation: Action is not recognized (cmd message)!");
    }
}

/**
 * Store CoM and feet map coordinates.
 * 
 * @param real
 */
void Navigation::storeMapCoordinates(const bool real) {
    updateVariablesFromCache();

    std::vector<unitree_legged_msgs::Cartesian> l_feetConfigurationMap;
    unitree_legged_msgs::Cartesian l_lf;
    l_lf.x = m_latestCoMPose.pose.pose.position.x + m_feetConfigurationCoM[0].x;
    l_lf.y = m_latestCoMPose.pose.pose.position.y + m_feetConfigurationCoM[0].y;
    l_lf.z = m_latestCoMPose.pose.pose.position.z + m_feetConfigurationCoM[0].z;
    unitree_legged_msgs::Cartesian l_fr;
    l_fr.x = m_latestCoMPose.pose.pose.position.x + m_feetConfigurationCoM[1].x;
    l_fr.y = m_latestCoMPose.pose.pose.position.y + m_feetConfigurationCoM[1].y;
    l_fr.z = m_latestCoMPose.pose.pose.position.z + m_feetConfigurationCoM[1].z;
    unitree_legged_msgs::Cartesian l_rl;
    l_rl.x = m_latestCoMPose.pose.pose.position.x + m_feetConfigurationCoM[2].x;
    l_rl.y = m_latestCoMPose.pose.pose.position.y + m_feetConfigurationCoM[2].y;
    l_rl.z = m_latestCoMPose.pose.pose.position.z + m_feetConfigurationCoM[2].z;
    unitree_legged_msgs::Cartesian l_rr;
    l_rr.x = m_latestCoMPose.pose.pose.position.x + m_feetConfigurationCoM[3].x;
    l_rr.y = m_latestCoMPose.pose.pose.position.y + m_feetConfigurationCoM[3].y;
    l_rr.z = m_latestCoMPose.pose.pose.position.z + m_feetConfigurationCoM[3].z;
    l_feetConfigurationMap.push_back(l_lf);
    l_feetConfigurationMap.push_back(l_fr);
    l_feetConfigurationMap.push_back(l_rl);
    l_feetConfigurationMap.push_back(l_rr);

    if (real) {
        m_realCoMPoses.push_back(m_latestCoMPose);
        m_realFeetPoses.push_back(l_feetConfigurationMap);
    }
    else {
        m_predictionInputCoM.push_back(m_latestCoMPose);
        m_predictionInputFeet.push_back(l_feetConfigurationMap);
    }
}

/**
 * Update variables from caches.
 */
void Navigation::updateVariablesFromCache() {
    // Update data from callback
    ros::spinOnce();

    // Store latest robot pose
    boost::shared_ptr<nav_msgs::Odometry const> l_latestCoMPose = m_robotPoseCache.getElemBeforeTime(ros::Time::now());
    m_latestCoMPose = *l_latestCoMPose;

    // Store latest feet poses w.r.t CoM
    try{
        auto l_latesLFFootPose = m_buffer.lookupTransform(ROBOT_REFERENCE_FRAME, "lf_foot", ros::Time(0));
        auto l_latestRFFootPose = m_buffer.lookupTransform(ROBOT_REFERENCE_FRAME, "rf_foot", ros::Time(0));
        auto l_latestLHFootPose = m_buffer.lookupTransform(ROBOT_REFERENCE_FRAME, "lh_foot", ros::Time(0));
        auto l_latestRHFootPose = m_buffer.lookupTransform(ROBOT_REFERENCE_FRAME, "rh_foot", ros::Time(0));

        // Clear latest feet poses w.r.t CoM
        m_feetConfigurationCoM.clear();

        unitree_legged_msgs::Cartesian l_latestLFFootCoMPose;
        l_latestLFFootCoMPose.x = l_latesLFFootPose.transform.translation.x;
        l_latestLFFootCoMPose.y = l_latesLFFootPose.transform.translation.y;
        l_latestLFFootCoMPose.z = l_latesLFFootPose.transform.translation.z;
        m_feetConfigurationCoM.push_back(l_latestLFFootCoMPose);

        unitree_legged_msgs::Cartesian l_latestRFFootCoMPose;
        l_latestRFFootCoMPose.x = l_latestRFFootPose.transform.translation.x;
        l_latestRFFootCoMPose.y = l_latestRFFootPose.transform.translation.y;
        l_latestRFFootCoMPose.z = l_latestRFFootPose.transform.translation.z;
        m_feetConfigurationCoM.push_back(l_latestRFFootCoMPose);

        unitree_legged_msgs::Cartesian l_latestLHFootCoMPose;
        l_latestLHFootCoMPose.x = l_latestLHFootPose.transform.translation.x;
        l_latestLHFootCoMPose.y = l_latestLHFootPose.transform.translation.y;
        l_latestLHFootCoMPose.z = l_latestLHFootPose.transform.translation.z;
        m_feetConfigurationCoM.push_back(l_latestLHFootCoMPose);

        unitree_legged_msgs::Cartesian l_latestRHFootCoMPose;
        l_latestRHFootCoMPose.x = l_latestRHFootPose.transform.translation.x;
        l_latestRHFootCoMPose.y = l_latestRHFootPose.transform.translation.y;
        l_latestRHFootCoMPose.z = l_latestRHFootPose.transform.translation.z;
        m_feetConfigurationCoM.push_back(l_latestRHFootCoMPose);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("Could not retrieve feet poses in CoM frame: %s",ex.what());
    }

    // Store latest feet forces
    boost::shared_ptr<gazebo_msgs::ContactsState const> l_latestFeetForces = 
        m_feetForcesCache.getElemBeforeTime(ros::Time::now());
    m_latestFeetForces = {static_cast<float>(l_latestFeetForces->states[0].wrenches[0].force.z),
                          static_cast<float>(l_latestFeetForces->states[1].wrenches[0].force.z),
                          static_cast<float>(l_latestFeetForces->states[2].wrenches[0].force.z),
                          static_cast<float>(l_latestFeetForces->states[3].wrenches[0].force.z)};
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

    // Plan path
    m_planner.plan(m_path, 
                   m_swingingFRRL, 
                   m_previousAction, 
                   m_previousVelocity,
                   m_latestCoMPose, 
                   m_goalMsg,
                   m_feetConfigurationCoM);

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
            stopCmdPublisher();
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

            // Update global variables from cache
            updateVariablesFromCache();
            
            // Set cmd to be sent
            setCmd(l_node.action, l_node.velocity);

            ROS_INFO_STREAM("Sending velocity: " << l_node.velocity);

            // Contact conditions
            bool l_lfInContact = false;
            bool l_lhInContact = false;
            bool l_rfInContact = false;
            bool l_rhInContact = false;
            unsigned int l_feetInContact = 0;
            bool l_swingingFeetOutOfContact = false;

            // Velocity command execution
            const ros::Time l_startTime = ros::Time::now();
            while (l_feetInContact < 3) {
                // Update global variables from cache
                updateVariablesFromCache();

                // Update callback data
                ros::spinOnce();

                // Get feet forces
                auto l_lfForceZ = m_latestFeetForces[0];
                auto l_rfForceZ = m_latestFeetForces[1];
                auto l_lhForceZ = m_latestFeetForces[2];
                auto l_rhForceZ = m_latestFeetForces[3];

                // Check for when swinging feet 
                // are out of contact with the ground
                if (!l_swingingFeetOutOfContact &&
                    ((l_lfForceZ < 5 && l_rhForceZ < 5) || (l_rfForceZ < 5 && l_lhForceZ < 5))) {
                    l_swingingFeetOutOfContact = true;
                    ROS_INFO_STREAM("Navigation: Feet got out of contact: " << l_lfForceZ << ", " << l_rfForceZ << ", "
                                                                            << l_lhForceZ << ", " << l_rhForceZ);
                }

                // Once feet are out of contact with
                // the ground check for when all feet
                // are once again back in contact
                if (l_swingingFeetOutOfContact) {
                    if (!l_lfInContact && l_lfForceZ >= 30) {
                        l_lfInContact = true;
                        l_feetInContact += 1;
                    }
                    if (!l_lhInContact && l_lhForceZ >= 30) {
                        l_lhInContact = true;
                        l_feetInContact += 1;
                    }
                    if (!l_rfInContact && l_rfForceZ >= 30) {
                        l_rfInContact = true;
                        l_feetInContact += 1;
                    }
                    if (!l_rhInContact && l_rhForceZ >= 30) {
                        l_rhInContact = true;
                        l_feetInContact += 1;
                    }
                }
            }

            ROS_INFO_STREAM("E.O.C: " << ros::Time::now().toSec() - l_startTime.toSec() << ", "
                                      << m_feetConfigurationCoM[0].z << ", " 
                                      << m_feetConfigurationCoM[1].z << ", "
                                      << m_feetConfigurationCoM[2].z << ", "
                                      << m_feetConfigurationCoM[3].z);

            // Add commanded velocity
            m_velocities.push_back(l_node.velocity);

            // Store predicted CoM and swinging feet
            // map coordinates for visualization purposes
            m_predictedCoMPoses.push_back(l_node.worldCoordinates);
            m_predictedFootsteps.push_back(l_node.feetConfiguration);

            // Store actual CoM and feet poses in map
            // coordinates for visualization purposes
            storeMapCoordinates(true);

            // Update planning variables
            m_previousAction = l_node.action;
            m_previousVelocity = l_node.velocity;
            if (m_feetConfigurationCoM[1].x < m_feetConfigurationCoM[0].x) {
                ROS_INFO_STREAM("FR and FL x relatively: " << m_feetConfigurationCoM[1].x << ", "
                                                           << m_feetConfigurationCoM[0].x);
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

        // Re-planning if goal not within reach
        if ((std::abs(m_latestCoMPose.pose.pose.position.x - m_goalMsg.pose.position.x) > 0.01 ||
             std::abs(m_latestCoMPose.pose.pose.position.y - m_goalMsg.pose.position.y) > 0.2) &&
            m_latestCoMPose.pose.pose.position.x <= m_goalMsg.pose.position.x) {
            // Store actual CoM and feet poses in map
            // coordinates for visualization purposes
            storeMapCoordinates(false);

            m_planner.plan(m_path, 
                           m_swingingFRRL, 
                           m_previousAction, 
                           m_previousVelocity,
                           m_latestCoMPose, 
                           m_goalMsg,
                           m_feetConfigurationCoM);
        } else {
            ROS_INFO("Navigation: Goal has been reached.");
            break;
        }

        // Get callback data
        ros::spinOnce();
    }
    
    // Stop cmd publisher
    stopCmdPublisher();

    // Print planner stats
    m_planner.stats();

    // Visualize CoM and feet results
    if (VISUALIZE) {
        ROS_INFO_STREAM("Publishing predicted and real CoM trajectories");
        publishRealCoMPath();
        publishPredictedCoMPath();

        ROS_INFO_STREAM("Publishing predicted footsteps");
        publishPredictedFootstepSequence();

        ROS_INFO_STREAM("Publishing real footsteps");
        publishRealFootstepSequence();
    }

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
 * Publish the latest
 * predicted footsteps
 */
void Navigation::publishOnlinePredictedFootsteps() {
    while (ros::ok()) {
        if (m_path.empty()) {
            continue;
        }

        // Counters
        int j = 0;

        // Populate marker array
        for (unsigned int i = 0; i < FOOTSTEP_HORIZON; i++) {
            visualization_msgs::MarkerArray l_onlineConfiguration;

            visualization_msgs::Marker l_predictionCommon;
            l_predictionCommon.header.stamp = ros::Time::now();
            l_predictionCommon.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
            l_predictionCommon.type = 2;
            l_predictionCommon.action = 0;
            l_predictionCommon.pose.orientation.x = 0;
            l_predictionCommon.pose.orientation.y = 0;
            l_predictionCommon.pose.orientation.z = 0;
            l_predictionCommon.pose.orientation.w = 1;
            l_predictionCommon.scale.x = 0.05;
            l_predictionCommon.scale.y = 0.035;
            l_predictionCommon.scale.z = 0.035;

            visualization_msgs::Marker l_predictedFL = l_predictionCommon;
            l_predictedFL.id = j++;
            l_predictedFL.color.r = 1;
            l_predictedFL.color.g = 0.501;
            l_predictedFL.color.b = 0;
            l_predictedFL.color.a = static_cast<float>(1 - (i * 0.1));
            l_predictedFL.pose.position.x = m_path[i].feetConfiguration.flMap.x;
            l_predictedFL.pose.position.y = m_path[i].feetConfiguration.flMap.y;
            l_predictedFL.pose.position.z = m_path[i].feetConfiguration.flMap.z;

            visualization_msgs::Marker l_predictedFR = l_predictionCommon;
            l_predictedFR.id = j++;
            l_predictedFR.color.r = 1;
            l_predictedFR.color.g = 0.4;
            l_predictedFR.color.b = 0.4;
            l_predictedFR.color.a = static_cast<float>(1 - (i * 0.1));
            l_predictedFR.pose.position.x = m_path[i].feetConfiguration.frMap.x;
            l_predictedFR.pose.position.y = m_path[i].feetConfiguration.frMap.y;
            l_predictedFR.pose.position.z = m_path[i].feetConfiguration.frMap.z;

            visualization_msgs::Marker l_predictedRL = l_predictionCommon;
            l_predictedRL.id = j++;
            l_predictedRL.id = j++;
            l_predictedRL.color.r = 0;
            l_predictedRL.color.g = 1;
            l_predictedRL.color.b = 0.501;
            l_predictedRL.color.a = static_cast<float>(1 - (i * 0.1));
            l_predictedRL.pose.position.x = m_path[i].feetConfiguration.rlMap.x;
            l_predictedRL.pose.position.y = m_path[i].feetConfiguration.rlMap.y;
            l_predictedRL.pose.position.z = m_path[i].feetConfiguration.rlMap.z;

            visualization_msgs::Marker l_predictedRR = l_predictionCommon;
            l_predictedRR.id = j++;
            l_predictedRR.id = j++;
            l_predictedRR.color.r = 0;
            l_predictedRR.color.g = 0.501;
            l_predictedRR.color.b = 1;
            l_predictedRR.color.a = static_cast<float>(1 - (i * 0.1));
            l_predictedRR.pose.position.x = m_path[i].feetConfiguration.rrMap.x;
            l_predictedRR.pose.position.y = m_path[i].feetConfiguration.rrMap.y;
            l_predictedRR.pose.position.z = m_path[i].feetConfiguration.rrMap.z;

            l_onlineConfiguration.markers.push_back(l_predictedFL);
            l_onlineConfiguration.markers.push_back(l_predictedFR);
            l_onlineConfiguration.markers.push_back(l_predictedRL);
            l_onlineConfiguration.markers.push_back(l_predictedRR);

            m_targetFeetConfigurationPublisher.publish(l_onlineConfiguration);
        }
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
        l_inputFootCommon.color.r = 1;
        l_inputFootCommon.color.g = 0;
        l_inputFootCommon.color.b = 0;
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
        l_realFLFootMarker.pose.position.x = m_realFeetPoses[i][0].x;
        l_realFLFootMarker.pose.position.y = m_realFeetPoses[i][0].y;
        l_realFLFootMarker.pose.position.z = m_realFeetPoses[i][0].z;

        visualization_msgs::Marker l_realFRFootMarker = l_realFootCommonMarker;
        l_realFRFootMarker.id = j++;
        l_realFRFootMarker.pose.position.x = m_realFeetPoses[i][1].x;
        l_realFRFootMarker.pose.position.y = m_realFeetPoses[i][1].y;
        l_realFRFootMarker.pose.position.z = m_realFeetPoses[i][1].z;

        visualization_msgs::Marker l_realRLFootMarker = l_realFootCommonMarker;
        l_realRLFootMarker.id = j++;
        l_realRLFootMarker.pose.position.x = m_realFeetPoses[i][2].x;
        l_realRLFootMarker.pose.position.y = m_realFeetPoses[i][2].y;
        l_realRLFootMarker.pose.position.z = m_realFeetPoses[i][2].z;

        visualization_msgs::Marker l_realRRFootMarker = l_realFootCommonMarker;
        l_realRRFootMarker.id = j++;
        l_realRRFootMarker.pose.position.x = m_realFeetPoses[i][3].x;
        l_realRRFootMarker.pose.position.y = m_realFeetPoses[i][3].y;
        l_realRRFootMarker.pose.position.z = m_realFeetPoses[i][3].z;

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