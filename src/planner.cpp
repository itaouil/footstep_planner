/*
 * planner.cpp
 *
 *  Created on: Aug 16, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#include <spatialite_private.h>
#include "planner.hpp"

/**
 * Constructor
 * @param nh
 */
Planner::Planner(ros::NodeHandle& nh): m_nh(nh)
{
    initialize();
}

/**
 * Destructor
 */
Planner::~Planner() = default;

/**
 * Planner initialization
 */
void Planner::initialize()
{
    // Sleep for sensor warm up
    ros::Duration(0.5).sleep();

    // Publishers
    m_vel_pub = m_nh.advertise<geometry_msgs::Twist>(VELOCITY_CMD_TOPIC, 1);

    // Build initial height map
    buildInitialHeightMap();
}

/**
 * Performs a 360 rotation to acquire
 * full height map to be used for planning
 */
void Planner::buildInitialHeightMap()
{
    ROS_INFO("Planner: Started rotation behaviour to acquire full height map");

    // Const values
    const int l_angle = 360;
    const double l_speed = 10;

    // Convert from angles to radians
    const double l_angular_speed = l_speed * 2 * M_PI / 360;
    const double l_relative_angle = l_angle * 2 * M_PI / 360;

    // Build Twist message to send
    geometry_msgs::Twist l_vel_msg;
    l_vel_msg.linear.x = 0;
    l_vel_msg.linear.y = 0;
    l_vel_msg.linear.z = 0;
    l_vel_msg.angular.x = 0;
    l_vel_msg.angular.y = 0;
    l_vel_msg.angular.z = l_angular_speed;

    // Setting current variables for distance calculus
    double l_current_angle = 0;
    double l_t0 = ros::Time::now().toSec();

    // Send velocity command
    while (l_current_angle < l_relative_angle)
    {
        // Publish message
        m_vel_pub.publish(l_vel_msg);

        // Compute current robot direction
        double l_t1 = ros::Time::now().toSec();
        l_current_angle = l_angular_speed * (l_t1 - l_t0);

        ros::spinOnce();
    }

    // Stop sending velocity commands
    l_vel_msg.angular.z = 0;
    m_vel_pub.publish(l_vel_msg);

    ROS_INFO("Planner: Rotation behaviour completed.");
}

void Planner::elevation_map_callback(const grid_map_msgs::GridMap::ConstPtr& msg)
{
    //TODO: process elevation map and make it ready for planning approach
}

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "aliengo_planner");
    ros::NodeHandle nodeHandle("~");

    // Start planner
    Planner planner(nodeHandle);

    // Spin
    ros::spin();
    return 0;
}

