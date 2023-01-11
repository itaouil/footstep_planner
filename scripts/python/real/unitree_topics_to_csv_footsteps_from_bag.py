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
from geometry_msgs.msg import TwistStamped, TransformStamped
from unitree_legged_msgs.msg import HighStateStamped, HighCmdStamped

# Global flags (footstep extraction)
first_footstep = True
prev_footstep_time = None
prev_footstep_flag = False

# Global parameters (footstep extraction)
publisher = None
cmd_cache = None
state_cache = None
vicon_cache = None
fl_max_height = 0
fr_max_height = 0
rl_max_height = 0
rr_max_height = 0
velocities = []
accelerometer = []

# Global variables
path = "/home/user/ros_ws"
file_object = open(path + "/data1.csv", "a")

# Output
output = []


def footstep_extraction(msg):
    # Globals
    global output
    global cmd_cache
    global state_cache
    global vicon_cache
    global file_object
    global velocities
    global accelerometer

    # Retrieve caches
    timestamp = rospy.Time.now()
    cmd = cmd_cache.getElemBeforeTime(timestamp)
    state = state_cache.getElemBeforeTime(timestamp)
    vicon = vicon_cache.getElemBeforeTime(timestamp)

    if not cmd or not state or not vicon:
        print("Skipping")
        return

    if not msg.data:
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
                      str(0) + "," +  # 39
                      str(0) + "," +  # 40
                      str(0) + "," +  # 41
                      str(0) + "," +  # 42
                      str(0) + "," +  # 43
                      str(0) + "," +  # 44
                      str(0) + "," +  # 45
                      str(0) + "," +  # 46
                      str(0) + "," +  # 47

                      str(state.imu.quaternion[0]) + "," +  # 48
                      str(state.imu.quaternion[1]) + "," +  # 49
                      str(state.imu.quaternion[2]) + "," +  # 50
                      str(state.imu.quaternion[3]) + "," +  # 51

                      str(state.imu.rpy[0]) + "," +  # 52
                      str(state.imu.rpy[1]) + "," +  # 53
                      str(state.imu.rpy[2]) + "," +  # 54

                      str(vicon.transform.translation.x) + "," +  # 55
                      str(vicon.transform.translation.y) + "," +  # 56
                      str(vicon.transform.translation.z) + "," +  # 57

                      str(vicon.transform.rotation.x) + "," +  # 58
                      str(vicon.transform.rotation.y) + "," +  # 59
                      str(vicon.transform.rotation.z) + "," +  # 60
                      str(vicon.transform.rotation.w) + "," +  # 61

                      str(velocities_np_mean[0]) + "," +  # 62
                      str(velocities_np_mean[1]) + "," +  # 63
                      str(velocities_np_mean[2]) + "," +  # 64
                      str(velocities_np_mean[3]) + "," +  # 65

                      str(accelerometer_np_mean[0]) + "," +  # 66
                      str(accelerometer_np_mean[1]) + "," +  # 67
                      str(accelerometer_np_mean[2]) + "," +  # 68

                      str(state.imu.accelerometer[0]) + "," +  # 69
                      str(state.imu.accelerometer[1]) + "," +  # 70
                      str(state.imu.accelerometer[2]) + "," +  # 71

                      str(0) + "," +  # 69
                      str(0) + "," +  # 70
                      str(0) + "," +  # 71
                      str(0) + "\n")  # 72

    velocities = []
    accelerometer = []


def main():
    # Globals
    global publisher
    global cmd_cache
    global state_cache
    global vicon_cache

    rospy.init_node('topics_sim_to_csv')

    cmd_sub = message_filters.Subscriber("/aliengo_bridge/twist_cmd", TwistStamped, queue_size=1)
    cmd_cache = message_filters.Cache(cmd_sub, 200)

    state_sub = message_filters.Subscriber("/aliengo_bridge/high_state", HighStateStamped, queue_size=1)
    state_cache = message_filters.Cache(state_sub, 200)

    vicon_sub = message_filters.Subscriber("/vicon/Jumpaolo/base", TransformStamped, queue_size=1)
    vicon_cache = message_filters.Cache(vicon_sub, 200)

    rospy.Subscriber("/footstep", Bool, footstep_extraction, queue_size=1)

    rospy.spin()


# Execute main
if __name__ == '__main__':
    main()
