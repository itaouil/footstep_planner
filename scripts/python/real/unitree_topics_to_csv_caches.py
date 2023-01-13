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
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
from unitree_legged_msgs.msg import HighStateStamped

# Global flags (footstep extraction)
first_footstep = True
prev_footstep_time = None
prev_footstep_flag = False

# Global parameters (footstep extraction)
velocities = []
publisher = None
state_cache = None
fl_max_height = 0
fr_max_height = 0
rl_max_height = 0
rr_max_height = 0
accelerometer = []

# Global variables
path = "/home/user/ros_ws"
file_object = open(path + "/accelerations.csv", "a")

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

    # Acquire front feet heights
    fl_height = footholds_msg.footPosition2Body[1].z
    fr_height = footholds_msg.footPosition2Body[0].z

    # Acquire front feet forces
    fl_force = footholds_msg.footForce[1]
    fr_force = footholds_msg.footForce[0]

    # Update recorded front feet heights
    fl_max_height = max(fl_max_height, fl_height)
    fr_max_height = max(fr_max_height, fr_height)

    # Footstep conditions check
    footstep_force_condition = fl_force > 20 and fr_force > 20
    footstep_height_condition = abs(fl_height - fr_height) < height_threshold

    # Check if footstep detected or not
    if footstep_force_condition and footstep_height_condition:
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
        print(fl_rr_moving, fr_rl_moving, fl_max_height, fr_max_height)
        clean_max_heights()

    publisher.publish(footstep)
    rospy.set_param("/feet_in_contact", footstep.data)
    return footstep.data, fl_rr_moving, fr_rl_moving


def live_extraction(cmd):
    # Globals
    global output
    global state_cache
    global velocities
    global file_object
    global accelerometer

    # Check if at this time a valid footstep is detected
    state = state_cache.getElemBeforeTime(cmd.header.stamp)
    is_valid_footstep, fl_rr_moving, fr_rl_moving = valid_footstep(state)

    if not is_valid_footstep:
        velocities.append([state.velocity[0], state.velocity[1], state.velocity[2], state.yawSpeed])
        accelerometer.append([state.imu.accelerometer[0], state.imu.accelerometer[1], state.imu.accelerometer[2]])
        return

    # Compute mean velocity
    velocities_np_mean = np.asarray(velocities).mean(axis=0)

    # Compute mean accelerometer data
    accelerometer_np_mean = np.asarray(accelerometer).mean(axis=0)

    file_object.write(str(time.time()) + "," +  # 0

                      str(cmd.twist.linear.x) + "," +  # 1
                      str(cmd.twist.linear.y) + "," +  # 2
                      str(cmd.twist.angular.z) + "," +  # 3

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

                      str(state.footForce[1]) + "," +  # 35
                      str(state.footForce[0]) + "," +  # 36
                      str(state.footForce[3]) + "," +  # 37
                      str(state.footForce[2]) + "," +  # 38

                      str(state.imu.quaternion[0]) + "," +  # 39
                      str(state.imu.quaternion[1]) + "," +  # 40
                      str(state.imu.quaternion[2]) + "," +  # 41
                      str(state.imu.quaternion[3]) + "," +  # 42

                      str(state.imu.rpy[0]) + "," +  # 43
                      str(state.imu.rpy[1]) + "," +  # 44
                      str(state.imu.rpy[2]) + "," +  # 45

                      str(velocities_np_mean[0]) + "," +  # 46
                      str(velocities_np_mean[1]) + "," +  # 47
                      str(velocities_np_mean[2]) + "," +  # 48
                      str(velocities_np_mean[3]) + "," +  # 49

                      str(accelerometer_np_mean[0]) + "," +  # 50
                      str(accelerometer_np_mean[1]) + "," +  # 51
                      str(accelerometer_np_mean[2]) + "," +  # 52

                      str(state.imu.accelerometer[0]) + "," +  # 53
                      str(state.imu.accelerometer[1]) + "," +  # 54
                      str(state.imu.accelerometer[2]) + "," +  # 55

                      str(fl_rr_moving) + "," +  # 56
                      str(fr_rl_moving) + "," +  # 57
                      str(2) + "\n")  # 58

    velocities = []
    accelerometer = []


def main():
    # Globals
    global publisher
    global state_cache

    print("Starting node")

    # Initialise node
    rospy.init_node('topics_sim_to_csv')

    rospy.set_param("/height_threshold", 0.05)
    rospy.set_param("/feet_in_contact", False)

    publisher = rospy.Publisher('footstep2', Bool, queue_size=1)

    state_sub = message_filters.Subscriber("/aliengo_bridge/high_state", HighStateStamped, queue_size=100)
    state_cache = message_filters.Cache(state_sub, 1000)

    rospy.Subscriber("/aliengo_bridge/twist_cmd", TwistStamped, live_extraction, queue_size=1)

    rospy.spin()


# Execute main
if __name__ == '__main__':
    main()
