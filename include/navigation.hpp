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
     * Perform rotation movement to
     * build initial height map if
     * required.
     */
    void buildInitialHeightMap();

    /**
     * Sets goal message and calls
     * planner to plan the path to
     * the goal.
     *
     * @param p_goalMsg
     */
    void goalCallback(const geometry_msgs::PoseStamped &p_goalMsg);

    /**
     * Plans a path to a target goal
     * using an elevation map (2.5D).
     *
     * @param p_goalMsg
     */
    void planPathToGoal();

    /**
     * Stopping behaviour.
     */
    void stopAction();

    /**
     * Stomping behaviour.
     */
    void stompAction();

    /**
     * Execute planned velocity commands.
     *
     * @param p_path
     * @param p_goalFound
     */
    void executeVelocityCommands(const std::vector<Node> &p_path, bool p_goalFound);

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

    //! ROS node handle
    ros::NodeHandle m_nh;

    //! ROS rate
    ros::Rate m_rate;

    //! TF variables
    tf2_ros::TransformListener &m_tf2;
    tf2_ros::Buffer &m_buffer;

    //! Planner object
    Planner m_planner;

    //! ROS subscribers
    ros::Subscriber m_goalSubscriber;

    //! ROS publishers
    ros::Publisher m_velocityPublisher;
    ros::Publisher m_realPathPublisher;
    ros::Publisher m_targetPathPublisher;
    ros::Publisher m_targetGoalPublisher;
    ros::Publisher m_realFeetConfigurationPublisher;
    ros::Publisher m_targetFeetConfigurationPublisher;

    //! Dynamic reconfigure
    dynamic_reconfigure::Config m_drConf;
    dynamic_reconfigure::ReconfigureRequest m_drSrvReq;
    dynamic_reconfigure::ReconfigureResponse m_drSrvRes;
    dynamic_reconfigure::DoubleParameter m_drDoubleParam;

    //! Feet transformed
    std::vector<visualization_msgs::MarkerArray> m_targetFootsteps;

    //! Odometry cache
    message_filters::Cache<wb_controller::ComTask> m_robotPoseCache;
    message_filters::Subscriber<wb_controller::ComTask> m_robotPoseSubscriber;

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

    //! Predicted CoM poses
    std::vector<World3D> m_predictedCoMPoses;

    //! Real CoM poses
    std::vector<wb_controller::ComTask> m_realCoMPoses;

    //! Predicted feet poses
    std::vector<Node> m_predictedFeetPoses;

    //! Real feet poses
    std::vector<std::vector<wb_controller::CartesianTask>> m_realFeetPoses;

    //! Goal message
    geometry_msgs::PoseStamped m_goalMsg;

    //! Current swinging pair
    bool m_swingingFRRL;

    //! Current action given to planner
    Action m_currentAction;

    //! Current velocity given to planner
    double m_currentVelocity;

    //! Re-planning thread
    std::thread m_thread;

    //! Re-planning boolean
    bool m_planPathToGoal;

    //! Mutex for processed elevation maps queue
    std::mutex m_mutex;
};