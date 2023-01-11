/*
 * navigation.hpp
 *
 *  Created on: Aug 16, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

// C++ general
#include <queue>
#include <cmath>

// ROS general
#include <ros/ros.h>
#include <message_filters/cache.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Dynamic Reconfigure
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>

// ROS messages
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <unitree_legged_msgs/Cartesian.h>
#include <geometry_msgs/TransformStamped.h>
#include <unitree_legged_msgs/HighStateStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Planner
#include "planner.hpp"

// Elevation map processor
#include "elevationMapProcessor.hpp"

// Config file
#include "config.hpp"

class Navigation
{
public:
    explicit Navigation(ros::NodeHandle&, tf2_ros::Buffer &, tf2_ros::TransformListener &);
    virtual ~Navigation();

private:
    /**
     * Threaded cmd publisher.
     */
    void cmdPublisher();

    /**
     * Start execution of
     * threaded high cmd 
     * publisher.
     */
    void startCmdPublisher();

    /**
     * Stop execution of
     * threaded high cmd 
     * publisher.
     */
    void stopCmdPublisher();

    /**
     * Construct cmd to send.
     *
     * @param p_action
     * @param p_velocity
     */
    void setCmd(const Action &p_action, const double &p_velocity);

    /**
     * Perform rotation movement to
     * build initial height map if
     * required.
     */
    void buildInitialHeightMap();

    /**
     * Publish predicted CoM path.
     */
    void publishRealCoMPath();

    /**
     * Update variables from caches.
     */
    void updateVariablesFromCache();

    /**
     * Store CoM and feet map coordinates.
     * 
     * @param type
     */
    void storeMapCoordinates(const bool real);

    /**
     * Execute planned velocity commands.
     */
    void executeHighLevelCommands();

    /**
     * Publish predicted CoM path.
     */
    void publishPredictedCoMPath();

    /**
     * Publish predicted and real
     * CoM and footstep sequence.
     */
    void publishRealFootstepSequence();

    /**
     * Publish predicted CoM and
     * footstep sequence.
     */
    void publishPredictedFootstepSequence();
    
    /**
     * Sets goal message and calls
     * planner to plan the path to
     * the goal.
     *
     * @param p_goalMsg
     */
    void goalCallback(const geometry_msgs::PoseStamped &p_goalMsg);

    /**
     * Publish the latest
     * predicted footsteps
     */
    void publishOnlinePredictedFootsteps();

    /**
     * ROS variables
     */

    //! ROS rate
    ros::Rate m_rate;

    //! ROS node handle
    ros::NodeHandle m_nh;

    //! Velocity command
    geometry_msgs::TwistStamped m_velCmd;

    //! TF variables
    tf2_ros::Buffer &m_buffer;
    tf2_ros::TransformListener &m_tf2;

    //! ROS subscribers
    ros::Subscriber m_goalSubscriber;

    //! ROS publishers
    ros::Publisher m_velocityPublisher;
    ros::Publisher m_realPathPublisher;
    ros::Publisher m_targetPathPublisher;
    ros::Publisher m_targetGoalPublisher;
    ros::Publisher m_realFeetConfigurationPublisher;
    ros::Publisher m_targetFeetConfigurationPublisher;

    /**
     * Planner variables
     */

    //! Planner object
    Planner m_planner;
    
    //! Planner variables
    bool m_swingingFRRL;
    Action m_previousAction;
    double m_previousVelocity;
    
    //! Path planned
    std::vector<Node> m_path;

    //! Previous path planned
    std::vector<Node> m_previousPath;

    //! Commanded velocities
    std::vector<float> m_velocities;
    
    //! Latest CoM pose
    nav_msgs::Odometry m_latestCoMPose;

    //! Goal message
    geometry_msgs::PoseStamped m_goalMsg;

    //! Latest feet forces
    std::vector<float> m_latestFeetForces;

    //! Latest feet poses w.r.t to the CoM
    std::vector<unitree_legged_msgs::Cartesian> m_feetConfigurationCoM;

    //! Robot odometry cache
    message_filters::Cache<nav_msgs::Odometry> m_robotPoseCache;
    message_filters::Subscriber<nav_msgs::Odometry> m_robotPoseSubscriber;

    //! T265 odometry cache
    message_filters::Cache<nav_msgs::Odometry> m_t265PoseCache;
    message_filters::Subscriber<nav_msgs::Odometry> m_t265PoseSubscriber;

    //! Feet forces cache
    message_filters::Cache<unitree_legged_msgs::HighStateStamped> m_highStateCache;
    message_filters::Subscriber<unitree_legged_msgs::HighStateStamped> m_highStateSubscriber;

    /**
     * Velocity commands variables
     */

    //! Threads
    std::thread m_cmdPubThread;
    std::thread m_predFeetThread;
    
    //! Cmd publisher thread variable
    bool m_startedCmdPublisher;

    //! File streaming object
    std::ofstream m_fileStream;

    //! Predicted and real CoM poses
    std::vector<World3D> m_predictedCoMPoses;
    std::vector<nav_msgs::Odometry> m_realCoMPoses;

    //! Predicted and real feet poses
    std::vector<FeetConfiguration> m_predictedFootsteps;
    std::vector<std::vector<unitree_legged_msgs::Cartesian>> m_realFeetPoses;

    //! CoM and feet prediction input
    std::vector<nav_msgs::Odometry> m_predictionInputCoM;
    std::vector<std::vector<unitree_legged_msgs::Cartesian>> m_predictionInputFeet;
};