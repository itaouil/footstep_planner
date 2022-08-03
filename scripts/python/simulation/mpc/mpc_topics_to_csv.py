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
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import TwistStamped

# Global parameters (footstep extraction)
publisher = None
robot_name = "aliengo"

# Params for feet extraction
lf_moving = True
rf_moving = True
lh_moving = True
rh_moving = True

# Global variables
path = f"/home/ilyass/workspace/catkin_ws/src/footstep_planner/data/dataset_sim"
file_object = open(path + "/forward_clockwise.csv", "a")


def live_extraction(cmd_msg,
                    odom_msg,
                    lf_foot_msg,
                    rf_foot_msg,
                    lh_foot_msg,
                    rh_foot_msg):
    global file_object
    global lf_moving
    global rf_moving
    global lh_moving
    global rh_moving

    print("Here")

    # Contact check
    if lf_foot.states and rf_foot.states and lh_foot.states and rh_foot.states:
        if lf_moving and rf_moving and lh_moving and rh_moving:
            return
    else:
        lf_moving = True if lf_foot.states else False
        rf_moving = True if rf_foot.states else False
        lh_moving = True if lh_foot.states else False
        rh_moving = True if rh_foot.states else False
        return

    linear_x = cmd_msg.twist.linear.x
    linear_y = cmd_msg.twist.linear.y
    angular_yaw = cmd_msg.twist.angular.z

    file_object.write(str(time.time()) + "," + # 0

                      str(linear_x) + "," + # 1
                      str(linear_y) + "," + # 2
                      str(angular_yaw) + "," + # 3

                      str(lf_foot_msg.states[0].contact_positions[0]) + "," +  # 4
                      str(lf_foot_msg.states[0].contact_positions[1]) + "," +  # 5
                      str(lf_foot_msg.states[0].contact_positions[2]) + "," +  # 6
                      str(rf_foot_msg.states[0].contact_positions[0]) + "," +  # 7
                      str(rf_foot_msg.states[0].contact_positions[1]) + "," +  # 8
                      str(rf_foot_msg.states[0].contact_positions[2]) + "," +  # 9
                      str(lh_foot_msg.states[0].contact_positions[0]) + "," +  # 10
                      str(lh_foot_msg.states[0].contact_positions[1]) + "," +  # 11
                      str(lh_foot_msg.states[0].contact_positions[2]) + "," +  # 12
                      str(rh_foot_msg.states[0].contact_positions[0]) + "," +  # 13
                      str(rh_foot_msg.states[0].contact_positions[1]) + "," +  # 14
                      str(rh_foot_msg.states[0].contact_positions[2]) + "," +  # 15

                      str(odom_msg.pose.pose.position.x) + "," +  # 16
                      str(odom_msg.pose.pose.position.y) + "," +  # 17
                      str(odom_msg.pose.pose.position.z) + "," +  # 18
                      str(odom_msg.pose.pose.orientation.x) + "," +  # 19
                      str(odom_msg.pose.pose.orientation.y) + "," +  # 20
                      str(odom_msg.pose.pose.orientation.z) + "," +  # 21
                      str(odom_msg.pose.pose.orientation.w) + "," +  # 22
                      str(odom_msg.twist.twist.linear.x) + "," +  # 23
                      str(odom_msg.twist.twist.linear.y) + "," +  # 24
                      str(odom_msg.twist.twist.linear.z) + "," +  # 25
                      str(odom_msg.twist.twist.angular.x) + "," + # 26
                      str(odom_msg.twist.twist.angular.y) + "," + # 27
                      str(odom_msg.twist.twist.angular.z) + "," +  # 28

                      str(lf_moving) + "," +  # 29
                      str(rf_moving) + "," +  # 30
                      str(lh_moving) + "," +  # 31
                      str(rh_moving) + "\n")


def main():
    # Globals
    global client
    global publisher

    # Init
    rospy.init_node('topics_sim_to_csv')
    rospy.set_param("/height_threshold", 0.003)
    rospy.set_param("/feet_in_contact", False)
    publisher = rospy.Publisher('footstep', Bool, queue_size=1)

    cmd_msg = message_filters.Subscriber(f"/cmd_vel", TwistStamped)
    odom_msg = message_filters.Subscriber(f"/{robot_name}/ground_truth", Odometry)
    lf_foot_msg = message_filters.Subscriber(f"/lf_foot_bumper", ContactsState)
    rf_foot_msg = message_filters.Subscriber(f"/rf_foot_bumper", ContactsState)
    lh_foot_msg = message_filters.Subscriber(f"/lh_foot_bumper", ContactsState)
    rh_foot_msg = message_filters.Subscriber(f"/rh_foot_bumper", ContactsState)

    ts = message_filters.TimeSynchronizer([cmd_msg,
                                           odom_msg,
                                           lf_foot_msg,
                                           rf_foot_msg,
                                           lh_foot_msg,
                                           rh_foot_msg], 10)

    ts.registerCallback(live_extraction)

    rospy.spin()

# Execute main
if __name__ == '__main__':
    main()