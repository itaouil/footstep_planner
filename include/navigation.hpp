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
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <wb_controller/ContactForces.h>
#include <wb_controller/CartesianTask.h>
#include <wb_controller/ComTask.h>
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
     * Stopping behaviour.
     */
    void halt();

    /**
     * Stomping behaviour.
     */
    void stomp();

    /**
     * Reset the robot configuration.
     */
    void resetConfiguration();

    /**
     * Threaded joy publisher.
     */
    void joyPublisher();

    /**
     * Start execution of
     * threaded joy publisher.
     */
    void startJoyPublisher();

    /**
     * Stop execution of
     * threaded joy publisher.
     */
    void stopJoyPublisher();

    /**
     * Construct joy command to send.
     *
     * @param p_action
     * @param p_velocity
     */
    void setJoyCommand(const Action &p_action, const double &p_velocity);

    /**
     * Perform rotation movement to
     * build initial height map if
     * required.
     */
    void buildInitialHeightMap();

    /**
     * Update variables from caches.
     */
    void updateVariablesFromCache();

    /**
     * Sets goal message and calls
     * planner to plan the path to
     * the goal.
     *
     * @param p_goalMsg
     */
    void goalCallback(const geometry_msgs::PoseStamped &p_goalMsg);

    /**
     * Execute planned velocity commands.
     */
    void executeHighLevelCommands();

    /**
     * Publish predicted CoM path.
     *
     * @param p_path
     */
    void publishRealCoMPath();

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
     *
     * @param p_path
     */
    void publishPredictedFootstepSequence();

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

    //! FL foot pose cache
    message_filters::Cache<wb_controller::CartesianTask> m_flFootPoseCache;
    message_filters::Subscriber<wb_controller::CartesianTask> m_flFootPoseSubscriber;

    //! FR foot pose cache
    message_filters::Cache<wb_controller::CartesianTask> m_frFootPoseCache;
    message_filters::Subscriber<wb_controller::CartesianTask> m_frFootPoseSubscriber;

    //! RL foot pose cache
    message_filters::Cache<wb_controller::CartesianTask> m_rlFootPoseCache;
    message_filters::Subscriber<wb_controller::CartesianTask> m_rlFootPoseSubscriber;

    //! RR foot pose cache
    message_filters::Cache<wb_controller::CartesianTask> m_rrFootPoseCache;
    message_filters::Subscriber<wb_controller::CartesianTask> m_rrFootPoseSubscriber;

    //! Feet contact forces cache
    message_filters::Cache<wb_controller::ContactForces> m_contactForcesCache;
    message_filters::Subscriber<wb_controller::ContactForces> m_contactForcesSubscriber;

    //! Goal message
    geometry_msgs::PoseStamped m_goalMsg;

    //! Predicted feet poses
    std::vector<FeetConfiguration> m_predictedFootsteps;

    //! Predicted CoM poses
    std::vector<World3D> m_predictedCoMPoses;

    //! Real CoM poses
    std::vector<nav_msgs::Odometry> m_realCoMPoses;

    //! Real feet poses
    std::vector<std::vector<wb_controller::CartesianTask>> m_realFeetPoses;

    //! CoM prediction input
    std::vector<nav_msgs::Odometry> m_predictionInputCoM;

    //! Feet prediction input
    std::vector<std::vector<wb_controller::CartesianTask>> m_predictionInputFeet;

    //! Current swinging pair
    bool m_swingingFRRL;

    //! Current action given to planner
    Action m_previousAction;

    //! Current velocity given to planner
    double m_previousVelocity;

    //! Latest robot pose
    boost::shared_ptr<nav_msgs::Odometry const> m_latestRobotPose;

    //! Latest contact forces message
    boost::shared_ptr<wb_controller::ContactForces const> m_latestContactForces;

    //! Latest relative feet poses
    boost::shared_ptr<wb_controller::CartesianTask const> m_latestFLFootPose;
    boost::shared_ptr<wb_controller::CartesianTask const> m_latestFRFootPose;
    boost::shared_ptr<wb_controller::CartesianTask const> m_latestRLFootPose;
    boost::shared_ptr<wb_controller::CartesianTask const> m_latestRRFootPose;

    //! Joy command
    sensor_msgs::Joy m_joy;

    //! Publish or not
    bool m_startedJoyPublisher;

    //! Thread object for joy publishing
    std::thread m_thread;

    //! Mutex shared multi-threaded variable
    std::mutex m_mutex;
};