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
import time
import rospy
import message_filters

# ROS msgs imports
from std_msgs.msg import Bool
from dls_messages.msg import mpc_signal_in
from dls_messages.msg import mpc_signal_out
from geometry_msgs.msg import TwistStamped

# Global flags (footstep extraction)
first_footstep = True
prev_footstep_time = None
prev_footstep_flag = False

# Global parameters (footstep extraction)
fl_force = 60
fr_force = 60
rl_force = 60
rr_force = 60
publisher = None
fl_max_height = 0
fr_max_height = 0
rl_max_height = 0
rr_max_height = 0
mpc_in_cache = None
mpc_out_cache = None

# Global variables
file_object = open("/home/ilyass/dls_ws/src/footstep_planner/data/accelerations.csv", "a")

# Output
output = []

def clean_values():
    global fl_force
    global fr_force
    global rl_force
    global rr_force
    global fl_max_height
    global fr_max_height
    global rl_max_height
    global rr_max_height

    fl_force = 60
    fr_force = 60
    rl_force = 60
    rr_force = 60

    fl_max_height = -100
    fr_max_height = -100
    rl_max_height = -100
    rr_max_height = -100


def valid_footstep(mpc_in_msg, mpc_out_msg):
    global fl_force
    global fr_force
    global rl_force
    global rr_force
    global publisher
    global fl_max_height
    global fr_max_height
    global rl_max_height
    global rr_max_height
    global first_footstep
    global prev_footstep_time
    global prev_footstep_flag

    # Footstep boolean message
    footstep = Bool()
    footstep.data = False

    # Acquire front feet heights
    fl_height = mpc_in_msg.xop[14]
    fr_height = mpc_in_msg.xop[17]
    rl_height = mpc_in_msg.xop[20]
    rr_height = mpc_in_msg.xop[23]

    # Update recorded front feet heights
    fl_max_height = max(fl_max_height, fl_height)
    fr_max_height = max(fr_max_height, fr_height)

    # Footstep conditions check
    contact_condition_1 = fl_force < 5 and mpc_out_msg.GRF[2] > 10 or rr_force < 5 and mpc_out_msg.GRF[11] > 10
    contact_condition_2 = fr_force < 5 and mpc_out_msg.GRF[5] > 10 or rl_force < 5 and mpc_out_msg.GRF[8] > 10

    fl_force = min(fl_force, mpc_out_msg.GRF[2])
    fr_force = min(fr_force, mpc_out_msg.GRF[5])
    rl_force = min(rl_force, mpc_out_msg.GRF[11])
    rr_force = min(rr_force, mpc_out_msg.GRF[8])

    # Check if footstep detected or not
    if contact_condition_1 and contact_condition_2:
        if first_footstep:
            footstep.data = True
            first_footstep = False
            prev_footstep_flag = True
            prev_footstep_time = time.time()
        else:
            if not prev_footstep_flag and not (time.time() - prev_footstep_time) < 0.2:
                footstep.data = True
                prev_footstep_flag = True
                prev_footstep_time = time.time()
    else:
        footstep.data = False
        prev_footstep_flag = False

    # Check which feet swung
    fl_rr_moving, fr_rl_moving = None, None
    if footstep.data:
        fl_rr_moving = True if fl_max_height > fr_max_height else False
        fr_rl_moving = True if fr_max_height > fl_max_height else False
        clean_values()

    publisher.publish(footstep)
    rospy.set_param("/feet_in_contact", footstep.data)
    return footstep.data, fl_rr_moving, fr_rl_moving


def live_extraction(cmd):
    # Globals
    global output
    global file_object
    global mpc_in_cache
    global mpc_out_cache

    mpc_in_msg = mpc_in_cache.getElemBeforeTime(cmd.header.stamp)
    mpc_out_msg = mpc_out_cache.getElemBeforeTime(cmd.header.stamp)
    is_valid_footstep, fl_rr_moving, fr_rl_moving = valid_footstep(mpc_in_msg, mpc_out_msg)

    if not is_valid_footstep:
        return

    file_object.write(str(time.time()) + "," +  # 0

                      # Velocity command info
                      str(cmd.twist.linear.x) + "," +  # 1
                      str(cmd.twist.linear.y) + "," +  # 2
                      str(cmd.twist.angular.z) + "," +  # 3

                      # Feet positions in the world frame
                      str(mpc_in_msg.xop[12]) + "," +  # 4
                      str(mpc_in_msg.xop[13]) + "," +  # 5
                      str(mpc_in_msg.xop[14]) + "," +  # 6
                      str(mpc_in_msg.xop[15]) + "," +  # 7
                      str(mpc_in_msg.xop[16]) + "," +  # 8
                      str(mpc_in_msg.xop[17]) + "," +  # 9
                      str(mpc_in_msg.xop[18]) + "," +  # 10
                      str(mpc_in_msg.xop[19]) + "," +  # 11
                      str(mpc_in_msg.xop[20]) + "," +  # 12
                      str(mpc_in_msg.xop[21]) + "," +  # 13
                      str(mpc_in_msg.xop[22]) + "," +  # 14
                      str(mpc_in_msg.xop[23]) + "," +  # 15

                      # CoM position in the world frame
                      str(mpc_in_msg.xop[0]) + "," +  # 16
                      str(mpc_in_msg.xop[1]) + "," +  # 17
                      str(mpc_in_msg.xop[2]) + "," +  # 18

                      # CoM velocity in the world frame
                      str(mpc_in_msg.xop[3]) + "," +  # 19
                      str(mpc_in_msg.xop[4]) + "," +  # 20
                      str(mpc_in_msg.xop[5]) + "," +  # 21

                      # Rotation matrix defining base to world rotation
                      str(mpc_in_msg.Rop[0]) + "," +  # 22
                      str(mpc_in_msg.Rop[1]) + "," +  # 23
                      str(mpc_in_msg.Rop[2]) + "," +  # 24
                      str(mpc_in_msg.Rop[3]) + "," +  # 25
                      str(mpc_in_msg.Rop[4]) + "," +  # 26
                      str(mpc_in_msg.Rop[5]) + "," +  # 27
                      str(mpc_in_msg.Rop[6]) + "," +  # 28
                      str(mpc_in_msg.Rop[7]) + "," +  # 29
                      str(mpc_in_msg.Rop[8]) + "," +  # 30

                      str(fl_rr_moving) + "," +  # 31
                      str(fr_rl_moving) + "\n")  # 32


def main():
    global publisher
    global mpc_in_cache
    global mpc_out_cache

    print("Starting node")

    rospy.init_node('topics_sim_to_csv')

    rospy.set_param("/feet_in_contact", False)

    publisher = rospy.Publisher('footstep', Bool, queue_size=5)

    rospy.Subscriber("/cmd_vel", TwistStamped, live_extraction, queue_size=1)

    mpc_out_sub = message_filters.Subscriber("/mpc/mpc_signal_out", mpc_signal_out, queue_size=10)
    mpc_out_cache = message_filters.Cache(mpc_out_sub, 1000)

    mpc_in_sub = message_filters.Subscriber("/aliengo/mpc_signal_in", mpc_signal_in, queue_size=10)
    mpc_in_cache = message_filters.Cache(mpc_in_sub, 1000)

    rospy.spin()

# Execute main
if __name__ == '__main__':
    main()
