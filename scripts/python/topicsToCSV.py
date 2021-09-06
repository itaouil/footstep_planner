#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Listen to simulation topics:
        * /odom:                                to retrieve robot odometry (i.e. position of CoM and current velocity)
        * /foot:                                to retrieve foot position w.r.t to the base
        * /cmd_vel:                             to get robot commanded velocity
        * /foot_contacts:                       to retrieve whether the foot is contact with the ground or not
        * /visual/FR_foot_contact/the_force:    force of FR foot
        * /visual/FL_foot_contact/the_force:    force of FL foot
        * /visual/RR_foot_contact/the_force:    force of RR foot
        * /visual/RL_foot_contact/the_force:    force of RL foot

    and logs the relevant data to a csv file.
"""

# General imports
import time
import rospy
import message_filters

# ROS msgs imports
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped, PoseArray, Twist

# Global path and file
path = "/home/itaouil/workspace/aliengo_ws/src/aliengo_navigation/data/dataset3"
file_object = open(path + "/motion.csv", "a")


def callback(cmd_vel_msg, foot_msg, force_fr_msg, force_fl_msg, force_rr_msg, force_rl_msg, odom_msg):
    global file_object

    file_object.write(str(time.time()) + "," +
                      str(cmd_vel_msg.linear.x) + "," +
                      str(cmd_vel_msg.linear.y) + "," +
                      str(cmd_vel_msg.linear.z) + "," +
                      str(cmd_vel_msg.angular.x) + "," +
                      str(cmd_vel_msg.angular.y) + "," +
                      str(cmd_vel_msg.angular.z) + "," +
                      str(foot_msg.poses[0].position.x) + "," +
                      str(foot_msg.poses[0].position.y) + "," +
                      str(foot_msg.poses[0].position.z) + "," +
                      str(foot_msg.poses[1].position.x) + "," +
                      str(foot_msg.poses[1].position.y) + "," +
                      str(foot_msg.poses[1].position.z) + "," +
                      str(foot_msg.poses[2].position.x) + "," +
                      str(foot_msg.poses[2].position.y) + "," +
                      str(foot_msg.poses[2].position.z) + "," +
                      str(foot_msg.poses[3].position.x) + "," +
                      str(foot_msg.poses[3].position.y) + "," +
                      str(foot_msg.poses[3].position.z) + "," +
                      str(force_fl_msg.wrench.force.x) + "," +
                      str(force_fl_msg.wrench.force.y) + "," +
                      str(force_fl_msg.wrench.force.z) + "," +
                      str(force_fr_msg.wrench.force.x) + "," +
                      str(force_fr_msg.wrench.force.y) + "," +
                      str(force_fr_msg.wrench.force.z) + "," +
                      str(force_rl_msg.wrench.force.x) + "," +
                      str(force_rl_msg.wrench.force.y) + "," +
                      str(force_rl_msg.wrench.force.z) + "," +
                      str(force_rr_msg.wrench.force.x) + "," +
                      str(force_rr_msg.wrench.force.y) + "," +
                      str(force_rr_msg.wrench.force.z) + "," +
                      str(odom_msg.pose.pose.position.x) + "," +
                      str(odom_msg.pose.pose.position.y) + "," +
                      str(odom_msg.pose.pose.position.z) + "," +
                      str(odom_msg.twist.twist.linear.x) + "," +
                      str(odom_msg.twist.twist.linear.y) + "," +
                      str(odom_msg.twist.twist.linear.z) + "," +
                      str(odom_msg.twist.twist.angular.x) + "," +
                      str(odom_msg.twist.twist.angular.y) + "," +
                      str(odom_msg.twist.twist.angular.z) + "\n")


def listener():
    # Initialise node
    rospy.init_node('topics_sim_to_csv')
    odom_sub = message_filters.Subscriber("/odom", Odometry)
    cmd_vel_sub = message_filters.Subscriber("/cmd_vel", Twist)
    foot_sub = message_filters.Subscriber("/foot_poses", PoseArray)
    force_fr_sub = message_filters.Subscriber("/visual/FR_foot_contact/the_force", WrenchStamped)
    force_fl_sub = message_filters.Subscriber("/visual/FL_foot_contact/the_force", WrenchStamped)
    force_rr_sub = message_filters.Subscriber("/visual/RR_foot_contact/the_force", WrenchStamped)
    force_rl_sub = message_filters.Subscriber("/visual/RL_foot_contact/the_force", WrenchStamped)

    ts = message_filters.ApproximateTimeSynchronizer(
        [cmd_vel_sub, foot_sub, force_fr_sub, force_fl_sub, force_rr_sub, force_rl_sub, odom_sub], 10, 0.03, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.spin()


# Execute main
if __name__ == '__main__':
    listener()
