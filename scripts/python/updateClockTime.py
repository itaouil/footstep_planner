#!/usr/bin/env python2.7
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
import rospy

# ROS msgs imports
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from wb_controller.msg import ContactForces, CartesianTask

# Global publishers
odom_publisher = None
cmd_vel_publisher = None
fl_trunk_publisher = None
fr_trunk_publisher = None
rl_trunk_publisher = None
rr_trunk_publisher = None
footholds_publisher = None


def odom_callback(msg):
    new_odom = msg
    new_odom.header.stamp = rospy.Time.now()
    odom_publisher.publish(new_odom)


def cmd_vel_callback(msg):
    new_cmd = msg
    new_cmd.header.stamp = rospy.Time.now()
    cmd_vel_publisher.publish(new_cmd)


def fl_trunk_callback(msg):
    new_fl = msg
    new_fl.header.stamp = rospy.Time.now()
    fl_trunk_publisher.publish(new_fl)


def fr_trunk_callback(msg):
    new_fr = msg
    new_fr.header.stamp = rospy.Time.now()
    fr_trunk_publisher.publish(new_fr)


def rl_trunk_callback(msg):
    new_rl = msg
    new_rl.header.stamp = rospy.Time.now()
    rl_trunk_publisher.publish(new_rl)


def rr_trunk_callback(msg):
    new_rr = msg
    new_rr.header.stamp = rospy.Time.now()
    rr_trunk_publisher.publish(new_rr)


def footholds_callback(msg):
    new_footholds = msg
    new_footholds.header.stamp = rospy.Time.now()
    footholds_publisher.publish(new_footholds)


def main():
    global odom_publisher
    global cmd_vel_publisher
    global fl_trunk_publisher
    global fr_trunk_publisher
    global rl_trunk_publisher
    global rr_trunk_publisher
    global footholds_publisher

    # Initialise node
    rospy.init_node('clock_update')

    # Create publishers
    odom_publisher = rospy.Publisher("/aliengo/ground_truth_republished", Odometry, queue_size=1)
    cmd_vel_publisher = rospy.Publisher("/aliengo/wb_controller/joy_republished", Joy, queue_size=1)
    fl_trunk_publisher = rospy.Publisher("/aliengo/wb_controller/lf_foot_republished", CartesianTask, queue_size=1)
    fr_trunk_publisher = rospy.Publisher("/aliengo/wb_controller/rf_foot_republished", CartesianTask, queue_size=1)
    rl_trunk_publisher = rospy.Publisher("/aliengo/wb_controller/lh_foot_republished", CartesianTask, queue_size=1)
    rr_trunk_publisher = rospy.Publisher("/aliengo/wb_controller/rh_foot_republished", CartesianTask, queue_size=1)
    footholds_publisher = rospy.Publisher("/aliengo/wb_controller/contact_forces_republished", ContactForces, queue_size=1)

    cmd_vel_sub = rospy.Subscriber("/aliengo/wb_controller/joy", Joy, cmd_vel_callback)
    odom_sub = rospy.Subscriber("/aliengo/ground_truth", Odometry, odom_callback)
    fl_trunk_sub = rospy.Subscriber("/aliengo/wb_controller/lf_foot", CartesianTask, fl_trunk_callback)
    fr_trunk_sub = rospy.Subscriber("/aliengo/wb_controller/rf_foot", CartesianTask, fr_trunk_callback)
    rl_trunk_sub = rospy.Subscriber("/aliengo/wb_controller/lh_foot", CartesianTask, rl_trunk_callback)
    rr_trunk_sub = rospy.Subscriber("/aliengo/wb_controller/rh_foot", CartesianTask, rr_trunk_callback)
    footholds_sub = rospy.Subscriber("/aliengo/wb_controller/contact_forces", ContactForces, footholds_callback)

    rospy.spin()


# Execute main
if __name__ == '__main__':
    main()
