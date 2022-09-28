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
from geometry_msgs.msg import TwistStamped
from unitree_legged_msgs.msg import HighStateStamped, HighCmdStamped

# Global flags (footstep extraction)
first_footstep = True
prev_footstep_time = None
prev_footstep_flag = False

# Global parameters (footstep extraction)
cmd_cache = None
publisher = None
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


def live_extraction(state):
    # Globals
    global output
    global cmd_cache
    global file_object

    # Check at this time a valid footstep is detected
    is_valid_footstep, fl_moving, fr_moving, rl_moving, rr_moving = valid_footstep(state)

    # Get synced command with cache
    stamp = cmd_cache.getLastestTime()
    cmd = cmd_cache.getElemBeforeTime(stamp)

    if not cmd:
        return

    # If not a valid footstep, skip callback
    if not is_valid_footstep:
        return

    file_object.write(str(time.time()) + "," +  # 0

                      str(cmd.twist.linear.x) + "," +  # 1
                      str(cmd.twist.linear.y) + "," +  # 2
                      str(cmd.twist.angular.z) + "," +     # 3

                      str(state.footPosition2Body[1].x) + "," +  # 4
                      str(state.footPosition2Body[1].y) + "," +  # 5
                      str(state.footPosition2Body[1].z) + "," +  # 6
                      str(state.footPosition2Body[0].x) + "," +  # 7
                      str(state.footPosition2Body[0].y) + "," +  # 8
                      str(state.footPosition2Body[0].z) + "," +  # 9
                      str(state.footPosition2Body[3].x) + "," +  # 10
                      str(state.footPosition2Body[3].y) + "," +  # 11
                      str(state.footPosition2Body[3].z) + "," +  # 12
                      str(state.footPosition2Body[2].x) + "," +  # 13
                      str(state.footPosition2Body[2].y) + "," +  # 14
                      str(state.footPosition2Body[2].z) + "," +  # 15

                      str(state.position[0]) + "," +  # 16
                      str(state.position[1]) + "," +  # 17
                      str(state.position[2]) + "," +  # 18

                      str(state.velocity[0]) + "," +  # 19
                      str(state.velocity[1]) + "," +  # 20
                      str(state.velocity[2]) + "," +  # 21
                      str(state.yawSpeed) + "," +  # 22

                      str(state.footSpeed2Body[1].x) + "," +  # 23
                      str(state.footSpeed2Body[1].y) + "," +  # 24
                      str(state.footSpeed2Body[1].z) + "," +  # 25
                      str(state.footSpeed2Body[0].x) + "," +  # 26
                      str(state.footSpeed2Body[0].y) + "," +  # 27
                      str(state.footSpeed2Body[0].z) + "," +  # 28
                      str(state.footSpeed2Body[3].x) + "," +  # 29
                      str(state.footSpeed2Body[3].y) + "," +  # 30
                      str(state.footSpeed2Body[3].z) + "," +  # 31
                      str(state.footSpeed2Body[2].x) + "," +  # 32
                      str(state.footSpeed2Body[2].y) + "," +  # 33
                      str(state.footSpeed2Body[2].z) + "," +  # 34

                      str(0) + "," +  # 35
                      str(0) + "," +  # 36
                      str(0) + "," +  # 37
                      str(0) + "," +  # 38
                      str(0) + "," +  # 39
                      str(0) + "," +  # 40
                      str(0) + "," +  # 41
                      str(0) + "," +  # 42
                      str(0) + "," +  # 43
                      str(0) + "," +  # 44
                      str(0) + "," +  # 45
                      str(0) + "," +  # 46
                      str(0) + "," +  # 47

                      str(state.imu.quaternion[0]) + "," + # 48
                      str(state.imu.quaternion[1]) + "," + # 49
                      str(state.imu.quaternion[2]) + "," + # 50
                      str(state.imu.quaternion[3]) + "," + # 51

                      str(state.imu.rpy[0]) + "," + # 52
                      str(state.imu.rpy[1]) + "," + # 53
                      str(state.imu.rpy[2]) + "," + # 54

                      str(fl_moving) + "," +  # 55
                      str(fr_moving) + "," +  # 56
                      str(rl_moving) + "," +  # 57
                      str(rr_moving) + "\n")  # 58


def main():
    # Globals
    global client
    global publisher
    global cmd_cache

    print("Starting node")

    # Initialise node
    rospy.init_node('topics_sim_to_csv')

    rospy.set_param("/height_threshold", 0.02)
    rospy.set_param("/feet_in_contact", False)

    publisher = rospy.Publisher('footstep2', Bool, queue_size=1)

    cmd_sub = message_filters.Subscriber("/aliengo_bridge/twist_cmd", TwistStamped, queue_size=1)
    cmd_cache = message_filters.Cache(cmd_sub, 1000)

    rospy.Subscriber("/aliengo_bridge/high_state", HighStateStamped, live_extraction, queue_size=1)

    rospy.spin()


# Execute main
if __name__ == '__main__':
    main()
