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
import numpy as np
import message_filters
import tf.transformations as tr

# ROS msgs imports
from nav_msgs.msg import Odometry
from dls_messages.msg import mpc_signal_in
from dls_messages.msg import mpc_signal_out
from unitree_legged_msgs.msg import Cartesian
from unitree_legged_msgs.msg import HighStateStamped

odom_pub = None
state_pub = None
mpc_in_cache = None
mpc_out_cache = None


def publish(mpc_in_msg):
    global  odom_pub
    global state_pub

    # Create odom and high state messages
    odom_msg = Odometry()
    state_msg = HighStateStamped()

    # Retrieve mpc out message
    mpc_out_msg = mpc_out_cache.getElemBeforeTime(mpc_in_msg.header.stamp)

    # Transform rotation matrix to quaternion
    R = [mpc_in_msg.Rop[0], mpc_in_msg.Rop[1], mpc_in_msg.Rop[2], 0,
         mpc_in_msg.Rop[3], mpc_in_msg.Rop[4], mpc_in_msg.Rop[5], 0,
         mpc_in_msg.Rop[6], mpc_in_msg.Rop[7], mpc_in_msg.Rop[8], 0,
         0, 0, 0, 1]
    R = np.asarray(R).reshape(4,4)
    Q = tr.quaternion_from_matrix(R)

    # Populate odometry message
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "world"
    odom_msg.child_frame_id = "base"
    odom_msg.pose.pose.position.x = mpc_in_msg.xop[0]
    odom_msg.pose.pose.position.y = mpc_in_msg.xop[1]
    odom_msg.pose.pose.position.z = mpc_in_msg.xop[2]
    odom_msg.twist.twist.linear.x = mpc_in_msg.xop[3]
    odom_msg.twist.twist.linear.y = mpc_in_msg.xop[4]
    odom_msg.twist.twist.linear.z = mpc_in_msg.xop[5]
    odom_msg.pose.pose.orientation.x = Q[0]
    odom_msg.pose.pose.orientation.y = Q[1]
    odom_msg.pose.pose.orientation.z = Q[2]
    odom_msg.pose.pose.orientation.w = Q[3]

    # Transform feet poses from world frame to base frame
    com_pose_world = np.asarray([mpc_in_msg.xop[0], mpc_in_msg.xop[1], mpc_in_msg.xop[2]])
    fl_pose_world = np.asarray([mpc_in_msg.xop[12], mpc_in_msg.xop[13], mpc_in_msg.xop[14]])
    fr_pose_world = np.asarray([mpc_in_msg.xop[15], mpc_in_msg.xop[16], mpc_in_msg.xop[17]])
    rl_pose_world = np.asarray([mpc_in_msg.xop[18], mpc_in_msg.xop[19], mpc_in_msg.xop[20]])
    rr_pose_world = np.asarray([mpc_in_msg.xop[21], mpc_in_msg.xop[22], mpc_in_msg.xop[23]])
    fl_pose_base = np.dot(R[:3, :3].T, fl_pose_world - com_pose_world).tolist()
    fr_pose_base = np.dot(R[:3, :3].T, fr_pose_world - com_pose_world).tolist()
    rl_pose_base = np.dot(R[:3, :3].T, rl_pose_world - com_pose_world).tolist()
    rr_pose_base = np.dot(R[:3, :3].T, rr_pose_world - com_pose_world).tolist()

    # Populate high state message
    state_msg.header.stamp = rospy.Time.now()
    state_msg.position[0] = mpc_in_msg.xop[0]
    state_msg.position[1] = mpc_in_msg.xop[1]
    state_msg.position[2] = mpc_in_msg.xop[2]
    state_msg.velocity[0] = mpc_in_msg.xop[3]
    state_msg.velocity[1] = mpc_in_msg.xop[4]
    state_msg.velocity[2] = 0.0
    state_msg.yawSpeed = mpc_in_msg.xop[5]
    state_msg.footPosition2Body[0] = Cartesian(fl_pose_base[0], fl_pose_base[1], fl_pose_base[2])
    state_msg.footPosition2Body[1] = Cartesian(fr_pose_base[0], fr_pose_base[1], fr_pose_base[2])
    state_msg.footPosition2Body[2] = Cartesian(rl_pose_base[0], rl_pose_base[1], rl_pose_base[2])
    state_msg.footPosition2Body[3] = Cartesian(rr_pose_base[0], rr_pose_base[1], rl_pose_base[2])
    state_msg.footForce[0] = mpc_out_msg.GRF[2]
    state_msg.footForce[1] = mpc_out_msg.GRF[5]
    state_msg.footForce[2] = mpc_out_msg.GRF[8]
    state_msg.footForce[3] = mpc_out_msg.GRF[11]

    odom_pub.publish(odom_msg)
    state_pub.publish(state_msg)

def main():
    global odom_pub
    global state_pub
    global mpc_in_cache
    global mpc_out_cache

    print("Starting sim to real topics publisher node.")

    rospy.init_node('topics_sim_to_csv')

    # Publishers
    odom_pub = rospy.Publisher('/aliengo_bridge/odom', Odometry, queue_size=1)
    state_pub = rospy.Publisher('/aliengo_bridge/high_state', HighStateStamped, queue_size=1)

    # Subscribers
    rospy.Subscriber("/aliengo/mpc_signal_in", mpc_signal_in, publish, queue_size=1)

    # Cache for the mpc out message
    mpc_out_sub = message_filters.Subscriber("/mpc/mpc_signal_out", mpc_signal_out, queue_size=1)
    mpc_out_cache = message_filters.Cache(mpc_out_sub, 1000)

    rospy.spin()

# Execute main
if __name__ == '__main__':
    main()
