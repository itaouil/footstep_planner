#!/usr/bin/env python3
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
from gazebo_msgs.msg import ContactsState

# Global publishers
lf_pub = None
rf_pub = None
lh_pub = None
rh_pub = None

def lf_callback(msg):
    lf_foot_bumper = msg
    lf_foot_bumper.header.stamp = rospy.Time.now()
    lf_pub.publish(lf_foot_bumper)


def rf_callback(msg):
    rf_foot_bumper = msg
    rf_foot_bumper.header.stamp = rospy.Time.now()
    rf_pub.publish(rf_foot_bumper)


def lh_callback(msg):
    lh_foot_bumper = msg
    lh_foot_bumper.header.stamp = rospy.Time.now()
    lh_pub.publish(lh_foot_bumper)


def rh_callback(msg):
    rh_foot_bumper = msg
    rh_foot_bumper.header.stamp = rospy.Time.now()
    rh_pub.publish(rh_foot_bumper)


def main():
    global lf_pub
    global rf_pub
    global lh_pub
    global rh_pub

    # Initialise node
    rospy.init_node('mcp_feet_bumper_republisher')

    # Create publishers
    lf_pub = rospy.Publisher("/aliengo/lf_foot_bumper2", ContactsState, queue_size=1)
    rf_pub = rospy.Publisher("/aliengo/rf_foot_bumper2", ContactsState, queue_size=1)
    lh_pub = rospy.Publisher("/aliengo/lh_foot_bumper2", ContactsState, queue_size=1)
    rh_pub = rospy.Publisher("/aliengo/rh_foot_bumper2", ContactsState, queue_size=1)

    lf_sub = rospy.Subscriber("/aliengo/lf_foot_bumper", ContactsState, lf_callback)
    rf_sub = rospy.Subscriber("/aliengo/rf_foot_bumper", ContactsState, rf_callback)
    lh_sub = rospy.Subscriber("/aliengo/lh_foot_bumper", ContactsState, lh_callback)
    rh_sub = rospy.Subscriber("/aliengo/rh_foot_bumper", ContactsState, rh_callback)

    rospy.spin()


# Execute main
if __name__ == '__main__':
    main()
