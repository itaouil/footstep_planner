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
from geometry_msgs.msg import TwistStamped
from unitree_legged_msgs.msg import HighState, HighCmd, HighStateStamped, HighCmdStamped

# Global publishers
odom_pub = None
high_cmd_pub = None
high_state_pub = None


def cmd_callback(msg):
    new_cmd = msg
    new_cmd.header.stamp = rospy.Time.now()
    high_cmd_pub.publish(new_cmd)


def state_callback(msg):
    new_state = msg
    new_state.header.stamp = rospy.Time.now()
    high_state_pub.publish(new_state)


def main():
    global odom_pub
    global high_cmd_pub
    global high_state_pub

    # Initialise node
    rospy.init_node('unitree_topics_republisher')

    # Create publishers
    high_cmd_pub = rospy.Publisher("/aliengo_bridge/twist_cmd2", TwistStamped, queue_size=1)
    high_state_pub = rospy.Publisher("/aliengo_bridge/high_state2", HighStateStamped, queue_size=1)

    high_cmd_sub = rospy.Subscriber("/aliengo_bridge/twist_cmd", TwistStamped, cmd_callback)
    high_state_sub = rospy.Subscriber("/aliengo_bridge/high_state", HighStateStamped, state_callback)

    rospy.spin()


# Execute main
if __name__ == '__main__':
    main()
