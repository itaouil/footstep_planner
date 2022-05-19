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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/Cartesian.h>
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
     * Publish online the
     * predicted footsteps.
     *
     * @param p_path
     */
    void publishOnlinePredictedFootsteps(std::vector<Node> &p_path);

    //! ROS rate
    ros::Rate m_rate;

    //! Planner object
    Planner m_planner;

    //! ROS node handle
    ros::NodeHandle m_nh;

    //! Path planned
    std::vector<Node> m_path;

    //! TF variables
    tf2_ros::TransformListener &m_tf2;
    tf2_ros::Buffer &m_buffer;

    //! ROS subscribers
    ros::Subscriber m_goalSubscriber;

    //! ROS publishers
    ros::Publisher m_velocityPublisher;
    ros::Publisher m_realPathPublisher;
    ros::Publisher m_targetPathPublisher;
    ros::Publisher m_targetGoalPublisher;
    ros::Publisher m_realFeetConfigurationPublisher;
    ros::Publisher m_targetFeetConfigurationPublisher;

    //! Feet transformed
    std::vector<visualization_msgs::MarkerArray> m_targetFootsteps;

    //! Odometry cache
    message_filters::Cache<nav_msgs::Odometry> m_robotPoseCache;
    message_filters::Subscriber<nav_msgs::Odometry> m_robotPoseSubscriber;

    //! High state cache
    message_filters::Cache<unitree_legged_msgs::HighStateStamped> m_highStateCache;
    message_filters::Subscriber<unitree_legged_msgs::HighStateStamped> m_highStateSubscriber;

    //! Commanded velocities
    std::vector<float> m_velocities;

    //! Goal message
    geometry_msgs::PoseStamped m_goalMsg;

    //! Predicted CoM and feet poses
    std::vector<World3D> m_predictedCoMPoses;
    std::vector<FeetConfiguration> m_predictedFootsteps;

    //! Real CoM and feet poses
    std::vector<nav_msgs::Odometry> m_realCoMPoses;
    std::vector<std::vector<unitree_legged_msgs::Cartesian>> m_realFeetPoses;

    //! CoM and feet prediction input
    std::vector<nav_msgs::Odometry> m_predictionInputCoM;
    std::vector<std::vector<unitree_legged_msgs::Cartesian>> m_predictionInputFeet;

    //! Planner variables
    bool m_swingingFRRL;
    Action m_previousAction;
    double m_previousVelocity;

    //! Latest CoM pose
    nav_msgs::Odometry m_latestCoMPose;

    //! Latest high state message
    boost::shared_ptr<unitree_legged_msgs::HighStateStamped const> m_latestHighState;
    
    //! Latest contact forces message
    std::vector<int16_t> m_latestContactForces;

    //! Latest relative feet poses
    unitree_legged_msgs::Cartesian m_latestFLFootPose;
    unitree_legged_msgs::Cartesian m_latestFRFootPose;
    unitree_legged_msgs::Cartesian m_latestRLFootPose;
    unitree_legged_msgs::Cartesian m_latestRRFootPose;

    //! High level command
    unitree_legged_msgs::HighCmd m_cmd;

    //! Cmd publisher thread variable
    bool m_startedCmdPublisher;

    //! Thread object for joy publishing
    std::thread m_thread;

    //! File streaming object
    std::ofstream m_fileStream;
};