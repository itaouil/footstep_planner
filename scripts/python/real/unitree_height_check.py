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

# ROS msgs imports
from std_msgs.msg import Bool
from unitree_legged_msgs.msg import HighStateStamped

# Global flags (footstep extraction)
first_footstep = True
prev_footstep_time = None
prev_footstep_flag = False

# Global parameters (footstep extraction)
publisher = None
fl_max_height = 0
fr_max_height = 0
rl_max_height = 0
rr_max_height = 0

# Global variables
path = "/home/dls"
file_object = open(path + "/forward_accelerations.csv", "a")


def clean_max_heights():
    global fl_max_height
    global fr_max_height
    global rl_max_height
    global rr_max_height

    fl_max_height = -100
    fr_max_height = -100
    rl_max_height = -100
    rr_max_height = -100


def valid_footstep(footholds_msg):
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

    # Get force threshold and height threshold
    height_threshold = rospy.get_param("/height_threshold")

    # Acquire heights
    fl_height = footholds_msg.footPosition2Body[1].z
    fr_height = footholds_msg.footPosition2Body[0].z
    rl_height = footholds_msg.footPosition2Body[3].z
    rr_height = footholds_msg.footPosition2Body[2].z

    # Update recorded max heights for each foot
    fl_max_height = max(fl_max_height, fl_height)
    fr_max_height = max(fr_max_height, fr_height)
    rl_max_height = max(rl_max_height, rl_height)
    rr_max_height = max(rr_max_height, rr_height)

    # Compute feet height difference booleans
    left_height_difference_in_range = abs(fl_height - rl_height) < height_threshold
    right_height_difference_in_range = abs(fr_height - rr_height) < height_threshold

    # Check if footstep detected or not
    if right_height_difference_in_range and left_height_difference_in_range:
        if first_footstep:
            footstep.data = True
            first_footstep = False
            prev_footstep_flag = True
            prev_footstep_time = time.time()
        else:
            if not prev_footstep_flag and not (time.time() - prev_footstep_time) < 0.2:
                print("Time: ", time.time() - prev_footstep_time)
                footstep.data = True
                prev_footstep_flag = True
                prev_footstep_time = time.time()
    else:
        prev_footstep_flag = False

    # Check that the feet motion that
    # brought the footstep is regular
    # (i.e. two feet on the ground and
    # and two in the air)
    fl_moving, fr_moving, rl_moving, rr_moving = None, None, None, None
    if footstep.data:
        # Compute booleans indicating which feet
        # are swinging based on height comparison
        # (swinging feet need to be diagonally
        # opposite)
        fl_moving = fl_max_height > fr_max_height
        fr_moving = fr_max_height > fl_max_height
        rl_moving = rl_max_height > rr_max_height
        rr_moving = rr_max_height > rl_max_height

        # Get max heights reached in the motion
        max_heights_sorted = sorted([fl_max_height, fr_max_height, rl_max_height, rr_max_height], reverse=True)
        swing1_max_height = max_heights_sorted[0]
        swing2_max_height = max_heights_sorted[1]

        # Compute booleans for swinging and max height conditions
        swinging_condition = fr_moving != fl_moving and rl_moving != rr_moving and fr_moving == rl_moving and fl_moving == rr_moving
        max_heights_condition = swing1_max_height > -0.3 and swing2_max_height > -0.3

        if not swinging_condition or not max_heights_condition:
            footstep.data = False
            if not max_heights_condition:
                print("Invalid max height condition")
                print(swing1_max_height, swing2_max_height)
            if not swinging_condition:
                print("Invalid swinging condition")
                print(fl_moving, fr_moving, rl_moving, rr_moving, fl_max_height, fr_max_height, rl_max_height,
                      rr_max_height)

        # Clean max height variable if footstep detected
        clean_max_heights()

    # Publish footstep detection boolean
    publisher.publish(footstep)

    rospy.set_param("/feet_in_contact", footstep.data)

    return footstep.data, fl_moving, fr_moving, rl_moving, rr_moving


def live_extraction(state):
    # Globals
    global file_object

    # Check at this time a valid footstep is detected
    valid_footstep(state)

def main():
    # Globals
    global client
    global publisher

    print("Starting node")

    # Initialise node
    rospy.init_node('topics_sim_to_csv')

    rospy.set_param("/height_threshold", 0.02)
    rospy.set_param("/feet_in_contact", False)

    publisher = rospy.Publisher('footstep', Bool, queue_size=1)
    rospy.Subscriber("/aliengo_bridge/high_state", HighStateStamped, live_extraction, queue_size=1)

    rospy.spin()


# Execute main
if __name__ == '__main__':
    main()
