#!/usr/bin/env python
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
import numpy as np
import time
import rospy
import message_filters

# ROS msgs imports
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from wb_controller.msg import ContactForces

# Publisher (footstep)
publisher = None
first_footstep = True
prev_footstep_flag = False
prev_footstep_time = 0.5

# Global variables
path = "/home/itaouil/workspace/code/aliengo_ws/src/aliengo_navigation/data/dataset4_wbc/live_extraction"
file_object = open(path + "/continuous_wbc_graiola_CoM.csv", "a")


def valid_footstep(cmd_vel_msg, foot_msg):
    global publisher
    global first_footstep
    global prev_footstep_time
    global prev_footstep_flag

    # Footstep boolean message
    footstep = Bool()
    footstep.data = False

    if any([abs(i) for i in cmd_vel_msg.axes]):
        # Get force threshold and height threshold
        height_threshold = rospy.get_param("/height_threshold")

        # Check if it is a footstep
        all_feet_in_contact = all([foot_msg.contact[0], foot_msg.contact[1], foot_msg.contact[2], foot_msg.contact[3]])
        back_height_difference_in_range = abs(abs(foot_msg.contact_positions[1].z) - abs(foot_msg.contact_positions[3].z)) < height_threshold
        front_height_difference_in_range = abs(abs(foot_msg.contact_positions[0].z) - abs(foot_msg.contact_positions[2].z)) < height_threshold

        if front_height_difference_in_range and back_height_difference_in_range:
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
            prev_footstep_flag = False
            print("Flag resetted due to conditions")
    else:
        print("Flag resetted due to command.")
        prev_footstep_flag = False

    publisher.publish(footstep)

    return footstep.data


def live_extraction(cmd_vel_msg, foot_msg, odom_msg):
    # Globals
    global file_object

    # If not a valid footstep, skip callback
    if not valid_footstep(cmd_vel_msg, foot_msg):
        return

    # Get currently commanded velocity
    current_velocity = rospy.get_param("/current_velocity")

    # Compute velocities
    linear_x = cmd_vel_msg.axes[1] * current_velocity
    linear_y = cmd_vel_msg.axes[0] * current_velocity if not cmd_vel_msg.axes[2] else 0.0
    angular_yaw = current_velocity * cmd_vel_msg.axes[2]

    # Remove - sign
    if not linear_x:
        linear_x = 0.0
    if not linear_y:
        linear_y = 0.0
    if not angular_yaw:
        angular_yaw = 0.0

    file_object.write(str(time.time()) + "," +

                      str(linear_x) + "," +
                      str(linear_y) + "," +
                      str(angular_yaw) + "," +

                      str(foot_msg.contact[0]) + "," +
                      str(foot_msg.contact[2]) + "," +
                      str(foot_msg.contact[1]) + "," +
                      str(foot_msg.contact[3]) + "," +

                      str(foot_msg.contact_positions[0].x) + "," + # 8
                      str(foot_msg.contact_positions[0].y) + "," +
                      str(foot_msg.contact_positions[0].z) + "," +
                      str(foot_msg.contact_positions[2].x) + "," +
                      str(foot_msg.contact_positions[2].y) + "," +
                      str(foot_msg.contact_positions[2].z) + "," +
                      str(foot_msg.contact_positions[1].x) + "," +
                      str(foot_msg.contact_positions[1].y) + "," +
                      str(foot_msg.contact_positions[1].z) + "," +
                      str(foot_msg.contact_positions[3].x) + "," +
                      str(foot_msg.contact_positions[3].y) + "," +
                      str(foot_msg.contact_positions[3].z) + "," + # 19

                      str(foot_msg.des_contact_forces[0].force.x) + "," +
                      str(foot_msg.des_contact_forces[0].force.y) + "," +
                      str(foot_msg.des_contact_forces[0].force.z) + "," +
                      str(foot_msg.des_contact_forces[2].force.x) + "," +
                      str(foot_msg.des_contact_forces[2].force.y) + "," +
                      str(foot_msg.des_contact_forces[2].force.z) + "," +
                      str(foot_msg.des_contact_forces[1].force.x) + "," +
                      str(foot_msg.des_contact_forces[1].force.y) + "," +
                      str(foot_msg.des_contact_forces[1].force.z) + "," +
                      str(foot_msg.des_contact_forces[3].force.x) + "," +
                      str(foot_msg.des_contact_forces[3].force.y) + "," +
                      str(foot_msg.des_contact_forces[3].force.z) + "," +

                      str(foot_msg.contact_forces[0].force.x) + "," +
                      str(foot_msg.contact_forces[0].force.y) + "," +
                      str(foot_msg.contact_forces[0].force.z) + "," +
                      str(foot_msg.contact_forces[2].force.x) + "," +
                      str(foot_msg.contact_forces[2].force.y) + "," +
                      str(foot_msg.contact_forces[2].force.z) + "," +
                      str(foot_msg.contact_forces[1].force.x) + "," +
                      str(foot_msg.contact_forces[1].force.y) + "," +
                      str(foot_msg.contact_forces[1].force.z) + "," +
                      str(foot_msg.contact_forces[3].force.x) + "," +
                      str(foot_msg.contact_forces[3].force.y) + "," +
                      str(foot_msg.contact_forces[3].force.z) + "," +

                      str(odom_msg.pose.pose.position.x) + "," + # 44
                      str(odom_msg.pose.pose.position.y) + "," +
                      str(odom_msg.pose.pose.position.z) + "," +
                      str(odom_msg.pose.pose.orientation.x) + "," + # 47
                      str(odom_msg.pose.pose.orientation.y) + "," +
                      str(odom_msg.pose.pose.orientation.z) + "," +
                      str(odom_msg.pose.pose.orientation.w) + "," +
                      str(odom_msg.twist.twist.linear.x) + "," + # 51
                      str(odom_msg.twist.twist.linear.y) + "," +
                      str(odom_msg.twist.twist.linear.z) + "," +
                      str(odom_msg.twist.twist.angular.x) + "," +
                      str(odom_msg.twist.twist.angular.y) + "," +
                      str(odom_msg.twist.twist.angular.z) + "\n") # 56


def odom_callback(cmd_vel_msg, foot_msg, odom_msg):
    # Globals
    global file_object

    # Get currently commanded velocity
    current_velocity = rospy.get_param("/current_velocity")

    # Compute velocities
    linear_x = cmd_vel_msg.axes[1] * current_velocity
    linear_y = cmd_vel_msg.axes[0] * current_velocity if not cmd_vel_msg.axes[2] else 0.0
    angular_yaw = current_velocity * cmd_vel_msg.axes[2]

    # Remove - sign
    if not linear_x:
        linear_x = 0.0
    if not linear_y:
        linear_y = 0.0
    if not angular_yaw:
        angular_yaw = 0.0

    file_object.write(str(time.time()) + "," +

                      str(linear_x) + "," +
                      str(linear_y) + "," +
                      str(angular_yaw) + "," +

                      str(foot_msg.contact[0]) + "," +
                      str(foot_msg.contact[2]) + "," +
                      str(foot_msg.contact[1]) + "," +
                      str(foot_msg.contact[3]) + "," +

                      str(foot_msg.contact_positions[0].x) + "," + # 8
                      str(foot_msg.contact_positions[0].y) + "," +
                      str(foot_msg.contact_positions[0].z) + "," +
                      str(foot_msg.contact_positions[2].x) + "," +
                      str(foot_msg.contact_positions[2].y) + "," +
                      str(foot_msg.contact_positions[2].z) + "," +
                      str(foot_msg.contact_positions[1].x) + "," +
                      str(foot_msg.contact_positions[1].y) + "," +
                      str(foot_msg.contact_positions[1].z) + "," +
                      str(foot_msg.contact_positions[3].x) + "," +
                      str(foot_msg.contact_positions[3].y) + "," +
                      str(foot_msg.contact_positions[3].z) + "," + # 19

                      str(foot_msg.des_contact_forces[0].force.x) + "," +
                      str(foot_msg.des_contact_forces[0].force.y) + "," +
                      str(foot_msg.des_contact_forces[0].force.z) + "," +
                      str(foot_msg.des_contact_forces[2].force.x) + "," +
                      str(foot_msg.des_contact_forces[2].force.y) + "," +
                      str(foot_msg.des_contact_forces[2].force.z) + "," +
                      str(foot_msg.des_contact_forces[1].force.x) + "," +
                      str(foot_msg.des_contact_forces[1].force.y) + "," +
                      str(foot_msg.des_contact_forces[1].force.z) + "," +
                      str(foot_msg.des_contact_forces[3].force.x) + "," +
                      str(foot_msg.des_contact_forces[3].force.y) + "," +
                      str(foot_msg.des_contact_forces[3].force.z) + "," +

                      str(foot_msg.contact_forces[0].force.x) + "," +
                      str(foot_msg.contact_forces[0].force.y) + "," +
                      str(foot_msg.contact_forces[0].force.z) + "," +
                      str(foot_msg.contact_forces[2].force.x) + "," +
                      str(foot_msg.contact_forces[2].force.y) + "," +
                      str(foot_msg.contact_forces[2].force.z) + "," +
                      str(foot_msg.contact_forces[1].force.x) + "," +
                      str(foot_msg.contact_forces[1].force.y) + "," +
                      str(foot_msg.contact_forces[1].force.z) + "," +
                      str(foot_msg.contact_forces[3].force.x) + "," +
                      str(foot_msg.contact_forces[3].force.y) + "," +
                      str(foot_msg.contact_forces[3].force.z) + "," +

                      str(odom_msg.pose.pose.position.x) + "," + # 44
                      str(odom_msg.pose.pose.position.y) + "," +
                      str(odom_msg.pose.pose.position.z) + "," +
                      str(odom_msg.pose.pose.orientation.x) + "," + # 47
                      str(odom_msg.pose.pose.orientation.y) + "," +
                      str(odom_msg.pose.pose.orientation.z) + "," +
                      str(odom_msg.pose.pose.orientation.w) + "," +
                      str(odom_msg.twist.twist.linear.x) + "," + # 51
                      str(odom_msg.twist.twist.linear.y) + "," +
                      str(odom_msg.twist.twist.linear.z) + "," +
                      str(odom_msg.twist.twist.angular.x) + "," +
                      str(odom_msg.twist.twist.angular.y) + "," +
                      str(odom_msg.twist.twist.angular.z) + "\n") # 56


def main():
    # Globals
    global client
    global publisher

    # Initialise node
    rospy.init_node('topics_sim_to_csv')

    # Set initial velocity
    rospy.set_param("/current_velocity", 0.0)
    rospy.set_param("/height_threshold", 0.002)

    publisher = rospy.Publisher('footstep', Bool, queue_size=1)

    odom_sub = message_filters.Subscriber("/aliengo/ground_truth", Odometry)
    cmd_vel_sub = message_filters.Subscriber("/aliengo/wb_controller/joy", Joy)
    footholds_sub = message_filters.Subscriber("/aliengo/wb_controller/contact_forces", ContactForces)

    ts = message_filters.ApproximateTimeSynchronizer([cmd_vel_sub, footholds_sub, odom_sub], 10, 0.01)
    ts.registerCallback(live_extraction)

    rospy.spin()


# Execute main
if __name__ == '__main__':
    main()
