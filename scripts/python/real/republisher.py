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
from unitree_legged_msgs.msg import HighState, HighCmd, HighStateStamped, HighCmdStamped

# Global publishers
odom_t265 = None
odom_pub = None
high_cmd_pub = None
high_state_pub = None


def odom_callback(msg):
    # global odom_t265
    # odom_t265 = msg
    new_odom = msg
    new_odom.header.stamp = rospy.Time.now()
    odom_pub.publish(new_odom)


def cmd_callback(msg):
    new_cmd = HighCmdStamped()
    new_cmd.header.stamp = rospy.Time.now()
    new_cmd.cmd = msg
    high_cmd_pub.publish(new_cmd)


def state_callback(msg):
    new_state = HighStateStamped()
    new_state.header.stamp = rospy.Time.now()
    new_state.state = msg
    high_state_pub.publish(new_state)

    # if odom_t265:
    #     odom_t265.header.stamp = rospy.Time.now()
    #     odom_pub.publish(odom_t265)


def main():
    global odom_pub
    global high_cmd_pub
    global high_state_pub

    # Initialise node
    rospy.init_node('real_system_republisher')

    # Create publishers
    odom_pub = rospy.Publisher("/t265/odom/sample2", Odometry, queue_size=1)
    high_cmd_pub = rospy.Publisher("/aliengo_bridge/high_cmd2", HighCmdStamped, queue_size=1)
    high_state_pub = rospy.Publisher("/aliengo_bridge/high_state2", HighStateStamped, queue_size=1)

    odom_sub = rospy.Subscriber("/t265/odom/sample", Odometry, odom_callback)
    high_cmd_sub = rospy.Subscriber("/aliengo_bridge/high_cmd", HighCmd, cmd_callback)
    high_state_sub = rospy.Subscriber("/aliengo_bridge/high_state", HighState, state_callback)

    rospy.spin()


# Execute main
if __name__ == '__main__':
    main()
