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
from unitree_legged_msgs.msg import HighStateStamped

# Global parameters (footstep extraction)
cmd_cache = None
publisher = None
state_cache = None

# Global variables
path = "/home/ilyass/workspace/catkin_ws/src/footstep_planner/data/dataset_real/gianpaolo/step_0.10/force"
file_object = open(path + "/sharp.csv", "a")

# Output
output = []


def write_to_file(valid_footstep):
    if not valid_footstep:
        return

    # Globals
    global output
    global cmd_cache
    global state_cache
    global file_object

    stamp_cmd = cmd_cache.getLastestTime()
    stamp_state = state_cache.getLastestTime()

    cmd = cmd_cache.getElemBeforeTime(stamp_cmd)
    state = state_cache.getElemBeforeTime(stamp_state)

    if not state or not cmd:
        return

        print(state.command[0], ", ", cmd.twist.linear.x)

    fl_moving = not state.footPosition2Body[1].x > state.footPosition2Body[0].x
    rl_moving = not state.footPosition2Body[3].x > state.footPosition2Body[2].x

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

                      str(state.imu.quaternion[0]) + "," + # 48
                      str(state.imu.quaternion[1]) + "," + # 49
                      str(state.imu.quaternion[2]) + "," + # 50
                      str(state.imu.quaternion[3]) + "," + # 51

                      str(state.imu.rpy[0]) + "," + # 52
                      str(state.imu.rpy[1]) + "," + # 53
                      str(state.imu.rpy[2]) + "," + # 54

                      str(fl_moving) + "," +  # 55
                      str(not fl_moving) + "," +  # 56
                      str(rl_moving) + "," +  # 57
                      str(not rl_moving) + "\n")  # 58


def main():
    # Globals
    global publisher
    global cmd_cache
    global state_cache

    print("Starting node")

    # Initialise node
    rospy.init_node('topics_sim_to_csv')

    state_sub = message_filters.Subscriber("/aliengo_bridge/high_state", HighStateStamped, queue_size=1)
    state_cache = message_filters.Cache(state_sub, 50)

    cmd_sub = message_filters.Subscriber("/aliengo_bridge/twist_cmd", TwistStamped, queue_size=1)
    cmd_cache = message_filters.Cache(cmd_sub, 50)

    rospy.Subscriber("/footstep", Bool, write_to_file, queue_size=1)

    rospy.spin()


# Execute main
if __name__ == '__main__':
    main()
