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
fl_force = 1000
fr_force = 1000
rl_force = 1000
rr_force = 1000
publisher = None
fl_max_height = 0
fr_max_height = 0
rl_max_height = 0
rr_max_height = 0
state_cache = None
prev_velocity = 0.0
prev_footstep_time = 0

# Global variables
path = "/home/ilyass/dls_ws/src/footstep_planner/data/real_monica"
file_object = open(path + "/sharp_accelerations.csv", "a")

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

    fl_force = -100
    fr_force = -100
    rl_force = -100
    rr_force = -100

    fl_max_height = -100
    fr_max_height = -100
    rl_max_height = -100
    rr_max_height = -100


def valid_footstep(footholds_msg, current_velocity):
    global fl_force
    global fr_force
    global rl_force
    global rr_force
    global publisher
    global fl_max_height
    global fr_max_height
    global rl_max_height
    global rr_max_height
    global prev_velocity
    global prev_footstep_time

    # Footstep boolean message
    footstep = Bool()
    footstep.data = False

    # Acquire front feet heights
    fl_height = footholds_msg.footPosition2Body[1].z
    fr_height = footholds_msg.footPosition2Body[0].z

    # Update recorded front feet heights
    fl_max_height = max(fl_max_height, fl_height)
    fr_max_height = max(fr_max_height, fr_height)

    # Update seen forces (only after half swing)
    if time.time() - prev_footstep_time > 0.15:
        fl_force = footholds_msg.footForce[1]
        fr_force = footholds_msg.footForce[0]
        rl_force = footholds_msg.footForce[3]
        rr_force = footholds_msg.footForce[2]

    # Footstep conditions check
    contact_condition_1 = fl_force > 20 or rr_force > 20
    contact_condition_2 = fr_force > 20 or rl_force > 20

    # Check if footstep detected or not
    if contact_condition_1 and contact_condition_2:
        if time.time() - prev_footstep_time > 0.2:
            footstep.data = True
            prev_footstep_time = time.time()

    # Check which feet swung
    fl_rr_moving, fr_rl_moving = None, None
    if footstep.data:
        fl_rr_moving = True if fl_max_height > fr_max_height else False
        fr_rl_moving = True if fr_max_height > fl_max_height else False
        clean_values()

    publisher.publish(footstep)
    rospy.set_param("/feet_in_contact", footstep.data)
    return footstep.data, fl_rr_moving, fr_rl_moving


def live_extraction(cmd, state):
    # Globals
    global output
    global state_cache
    global file_object

    print("Here")

    # Check if at this time a valid footstep is detected
    #state = state_cache.getElemBeforeTime(cmd.header.stamp)
    is_valid_footstep, fl_rr_moving, fr_rl_moving = valid_footstep(state, cmd.twist.linear.x)

    if not is_valid_footstep:
        return

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

                      str(fl_rr_moving) + "," +  # 46
                      str(fr_rl_moving) + "\n")  # 47


def main():
    # Globals
    global publisher
    global state_cache

    rospy.init_node('topics_sim_to_csv')
    rospy.set_param("/feet_in_contact", False)

    publisher = rospy.Publisher('footstep2', Bool, queue_size=1)

    # Sync state and cmd message
    cmd_sub = message_filters.Subscriber('/aliengo_bridge/twist_cmd', TwistStamped)
    state_sub = message_filters.Subscriber('/aliengo_bridge/high_state', HighStateStamped)

    ts = message_filters.ApproximateTimeSynchronizer([cmd_sub, state_sub], 10, 0.1)
    ts.registerCallback(live_extraction)
    rospy.spin()

    rospy.spin()


# Execute main
if __name__ == '__main__':
    main()
