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
import numpy as np
import message_filters

# ROS msgs imports
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import TwistStamped
from unitree_legged_msgs.msg import HighStateStamped, HighCmdStamped

# Global flags (footstep extraction)
first_footstep = True
prev_footstep_time = None
prev_footstep_flag = False

# Global parameters (footstep extraction)
cmd_cache = None
publisher = None
publisher_raibert = None
fl_max_height = 0
fr_max_height = 0
rl_max_height = 0
rr_max_height = 0

# Global variables
path = "/home/ilyass/workspace/catkin_ws/src/footstep_planner/data/dataset_real/gianpaolo/step_0.10"
file_object = open(path + "/forward_accelerations_sharp.csv", "a")

# Output
output = []


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
    front_height_difference_in_range = abs(fl_height - fr_height) < height_threshold
    hind_height_difference_in_range = abs(rl_height - rr_height) < height_threshold
    
    # Check if footstep detected or not
    if hind_height_difference_in_range and front_height_difference_in_range:
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
        footstep.data = False
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
        max_heights_condition = swing1_max_height > -0.30 and swing2_max_height > -0.30

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
    # if footstep.data:
    #     print("Here: ", abs(fl_height - fr_height), abs(rl_height - rr_height))

    publisher.publish(footstep)

    rospy.set_param("/feet_in_contact", footstep.data)

    return footstep.data, fl_moving, fr_moving, rl_moving, rr_moving


def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.

    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3)

    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix.
             This rotation matrix converts a point in the local reference
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[3]
    q1 = Q[0]
    q2 = Q[1]
    q3 = Q[2]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix


def live_extraction(state):
    global publisher_raibert

    # Get synced command with cache
    stamp = cmd_cache.getLastestTime()
    cmd = cmd_cache.getElemBeforeTime(stamp)

    if not cmd:
        return

    R = quaternion_rotation_matrix(state.imu.quaternion)
    hip_projection = np.array([[0.2399, 0.1340, 0]]).T
    predicted_fl = 0.5 * R.T @ np.array([state.velocity]).T * 0.35 + hip_projection - 2*np.pi*(0.35/9.81)**0.5*(np.array([cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z]).T - np.array([state.velocity]).T)
    publisher_raibert.publish(predicted_fl[0, 0])


def main():
    # Globals
    global client
    global publisher
    global cmd_cache
    global publisher_raibert

    print("Starting node")

    # Initialise node
    rospy.init_node('topics_sim_to_csv')

    rospy.set_param("/height_threshold", 0.02)
    rospy.set_param("/feet_in_contact", False)

    publisher = rospy.Publisher('footstep2', Bool, queue_size=1)

    publisher_raibert = rospy.Publisher('predicted_fl_x', Float32, queue_size=1)

    cmd_sub = message_filters.Subscriber("/aliengo_bridge/twist_cmd", TwistStamped, queue_size=1)
    cmd_cache = message_filters.Cache(cmd_sub, 1000)

    rospy.Subscriber("/aliengo_bridge/high_state", HighStateStamped, live_extraction, queue_size=1)

    rospy.spin()


# Execute main
if __name__ == '__main__':
    main()
