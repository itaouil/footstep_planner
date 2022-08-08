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
lf_rh_moving = True
rf_lh_moving = True

# Global variables
path = f"/home/ilyass/workspace/catkin_ws/src/footstep_planner/data/dataset_sim"
file_object = open(path + "/mpc_forward.csv", "a")


def live_extraction(cmd_msg,
                    odom_msg,
                    lf_foot_msg,
                    rf_foot_msg,
                    lh_foot_msg,
                    rh_foot_msg):
    global file_object
    global lf_rh_moving
    global rf_lh_moving

    # Contact check
    if not (lf_foot_msg.states and rf_foot_msg.states and lh_foot_msg.states and rh_foot_msg.states):
        lf_rh_moving = False if (lf_foot_msg.states and rh_foot_msg.states) else True
        rf_lh_moving = False if (rf_foot_msg.states and lh_foot_msg.states) else True
        publisher.publish(False)
        return
    
    # Make sure only one 
    # diagonal is moving
    if lf_rh_moving and rf_lh_moving:
        print("Two diagonals moving")
        publisher.publish(False)
        return
    
    publisher.publish(True)

    linear_x = cmd_msg.twist.linear.x
    linear_y = cmd_msg.twist.linear.y
    angular_yaw = cmd_msg.twist.angular.z
Contact check
    file_object.write(str(time.time()) + "," + # 0

                      str(linear_x) + "," + # 1
                      str(linear_y) + "," + # 2
                      str(angular_yaw) + "," + # 3

                      str(lf_foot_msg.states[0].contact_positions[0].x) + "," +  # 4
                      str(lf_foot_msg.states[0].contact_positions[0].y) + "," +  # 5
                      str(lf_foot_msg.states[0].contact_positions[0].z) + "," +  # 6
                      str(rf_foot_msg.states[0].contact_positions[0].x) + "," +  # 7
                      str(rf_foot_msg.states[0].contact_positions[0].y) + "," +  # 8
                      str(rf_foot_msg.states[0].contact_positions[0].z) + "," +  # 9
                      str(lh_foot_msg.states[0].contact_positions[0].x) + "," +  # 10
                      str(lh_foot_msg.states[0].contact_positions[0].y) + "," +  # 11
                      str(lh_foot_msg.states[0].contact_positions[0].z) + "," +  # 12
                      str(rh_foot_msg.states[0].contact_positions[0].x) + "," +  # 13
                      str(rh_foot_msg.states[0].contact_positions[0].y) + "," +  # 14
                      str(rh_foot_msg.states[0].contact_positions[0].z) + "," +  # 15

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

                      str(lf_rh_moving) + "," +  # 29
                      str(rf_lh_moving) + "\n") # 30


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
    lf_foot_msg = message_filters.Subscriber(f"/{robot_name}/lf_foot_bumper2", ContactsState)
    rf_foot_msg = message_filters.Subscriber(f"/{robot_name}/rf_foot_bumper2", ContactsState)
    lh_foot_msg = message_filters.Subscriber(f"/{robot_name}/lh_foot_bumper2", ContactsState)
    rh_foot_msg = message_filters.Subscriber(f"/{robot_name}/rh_foot_bumper2", ContactsState)

    ts = message_filters.ApproximateTimeSynchronizer([cmd_msg,
                                                      odom_msg,
                                                      lf_foot_msg,
                                                      rf_foot_msg,
                                                      lh_foot_msg,
                                                      rh_foot_msg], 10, 0.3)

    ts.registerCallback(live_extraction)

    rospy.spin()

# Execute main
if __name__ == '__main__':
    main()