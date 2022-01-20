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
import time
import rospy
import message_filters

# ROS msgs imports
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from wb_controller.msg import ContactForces, CartesianTask

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
path = "/home/itaouil/workspace/code/thesis_ws/src/footstep_planner/data/dataset4_wbc/live_extraction/step_0.06/gt/fwd_only"
file_object = open(path + "/continuous_08.csv", "a")


def clean_max_heights():
    global fl_max_height
    global fr_max_height
    global rl_max_height
    global rr_max_height

    fl_max_height = -100
    fr_max_height = -100
    rl_max_height = -100
    rr_max_height = -100


def valid_footstep(cmd_vel_msg, footholds_msg):
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

    if any([abs(i) for i in cmd_vel_msg.buttons]):
        # Get force threshold and height threshold
        height_threshold = rospy.get_param("/height_threshold")

        # Acquire heights
        fl_height = footholds_msg.contact_positions[0].z
        fr_height = footholds_msg.contact_positions[2].z
        rl_height = footholds_msg.contact_positions[1].z
        rr_height = footholds_msg.contact_positions[3].z

        # Update recorded max heights for each foot
        fl_max_height = max(fl_max_height, fl_height)
        fr_max_height = max(fr_max_height, fr_height)
        rl_max_height = max(rl_max_height, rl_height)
        rr_max_height = max(rr_max_height, rr_height)

        # Compute feet height difference booleans
        back_height_difference_in_range = abs(rl_height - rr_height) < height_threshold
        front_height_difference_in_range = abs(fl_height - fr_height) < height_threshold

        # Check if footstep detected or not
        if front_height_difference_in_range and back_height_difference_in_range:
            if first_footstep:
                footstep.data = True
                first_footstep = False
                prev_footstep_flag = True
                prev_footstep_time = time.time()
            else:
                if not prev_footstep_flag and not (time.time() - prev_footstep_time) < 0.3:
                    print("Time: ", time.time() - prev_footstep_time)
                    footstep.data = True
                    prev_footstep_flag = True
                    prev_footstep_time = time.time()
        else:
            prev_footstep_flag = False
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
        swinging_condition = fr_moving != fl_moving
        max_heights_condition = swing1_max_height > -0.39 and swing2_max_height > -0.39

        if not swinging_condition or not max_heights_condition:
            footstep.data = False
            if not swinging_condition:
                print("Invalid swinging condition")
                print(fl_moving, fr_moving, rl_moving, rr_moving)

        # Clean max height variable if footstep detected
        clean_max_heights()

    # Publish footstep detection boolean
    publisher.publish(footstep)

    rospy.set_param("/feet_in_contact", footstep.data)

    return footstep.data, fl_moving, fr_moving, rl_moving, rr_moving


def live_extraction(odom_msg,
                    cmd_vel_msg,
                    fl_trunk_msg,
                    fr_trunk_msg,
                    rl_trunk_msg,
                    rr_trunk_msg,
                    footholds_msg):
    # Globals
    global file_object

    # Check at this time a valid footstep is detected
    is_valid_footstep, fl_moving, fr_moving, rl_moving, rr_moving = valid_footstep(cmd_vel_msg, footholds_msg)

    # If not a valid footstep, skip callback
    if not is_valid_footstep:
        return

    # Compute velocities
    linear_x = cmd_vel_msg.axes[1]
    linear_y = cmd_vel_msg.axes[0]
    angular_yaw = cmd_vel_msg.axes[3]

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

                      str(footholds_msg.contact[0]) + "," +
                      str(footholds_msg.contact[2]) + "," +
                      str(footholds_msg.contact[1]) + "," +
                      str(footholds_msg.contact[3]) + "," +

                      str(fl_trunk_msg.pose_actual.position.x) + "," +  # 8
                      str(fl_trunk_msg.pose_actual.position.y) + "," +  # 9
                      str(fl_trunk_msg.pose_actual.position.z) + "," +
                      str(fr_trunk_msg.pose_actual.position.x) + "," +  # 11
                      str(fr_trunk_msg.pose_actual.position.y) + "," +  # 12
                      str(fr_trunk_msg.pose_actual.position.z) + "," +
                      str(rl_trunk_msg.pose_actual.position.x) + "," +  # 14
                      str(rl_trunk_msg.pose_actual.position.y) + "," +  # 15
                      str(rl_trunk_msg.pose_actual.position.z) + "," +
                      str(rr_trunk_msg.pose_actual.position.x) + "," +  # 17
                      str(rr_trunk_msg.pose_actual.position.y) + "," +  # 18
                      str(rr_trunk_msg.pose_actual.position.z) + "," +  # 19

                      str(footholds_msg.des_contact_forces[0].force.x) + "," +
                      str(footholds_msg.des_contact_forces[0].force.y) + "," +
                      str(footholds_msg.des_contact_forces[0].force.z) + "," +
                      str(footholds_msg.des_contact_forces[2].force.x) + "," +
                      str(footholds_msg.des_contact_forces[2].force.y) + "," +
                      str(footholds_msg.des_contact_forces[2].force.z) + "," +
                      str(footholds_msg.des_contact_forces[1].force.x) + "," +
                      str(footholds_msg.des_contact_forces[1].force.y) + "," +
                      str(footholds_msg.des_contact_forces[1].force.z) + "," +
                      str(footholds_msg.des_contact_forces[3].force.x) + "," +
                      str(footholds_msg.des_contact_forces[3].force.y) + "," +
                      str(footholds_msg.des_contact_forces[3].force.z) + "," +

                      str(footholds_msg.contact_forces[0].force.x) + "," +
                      str(footholds_msg.contact_forces[0].force.y) + "," +
                      str(footholds_msg.contact_forces[0].force.z) + "," +
                      str(footholds_msg.contact_forces[2].force.x) + "," +
                      str(footholds_msg.contact_forces[2].force.y) + "," +
                      str(footholds_msg.contact_forces[2].force.z) + "," +
                      str(footholds_msg.contact_forces[1].force.x) + "," +
                      str(footholds_msg.contact_forces[1].force.y) + "," +
                      str(footholds_msg.contact_forces[1].force.z) + "," +
                      str(footholds_msg.contact_forces[3].force.x) + "," +
                      str(footholds_msg.contact_forces[3].force.y) + "," +
                      str(footholds_msg.contact_forces[3].force.z) + "," +

                      str(odom_msg.pose.pose.position.x) + "," +  # 44
                      str(odom_msg.pose.pose.position.y) + "," +
                      str(odom_msg.pose.pose.position.z) + "," +
                      str(odom_msg.pose.pose.orientation.x) + "," +  # 47
                      str(odom_msg.pose.pose.orientation.y) + "," +
                      str(odom_msg.pose.pose.orientation.z) + "," +
                      str(odom_msg.pose.pose.orientation.w) + "," +
                      str(odom_msg.twist.twist.linear.x) + "," +  # 51
                      str(odom_msg.twist.twist.linear.y) + "," +
                      str(odom_msg.twist.twist.linear.z) + "," +
                      str(odom_msg.twist.twist.angular.x) + "," +
                      str(odom_msg.twist.twist.angular.y) + "," +
                      str(odom_msg.twist.twist.angular.z) + "," +  # 56

                      str(fl_moving) + "," +  # 57
                      str(fr_moving) + "," +
                      str(rl_moving) + "," +
                      str(rr_moving) + "\n")


def main():
    # Globals
    global client
    global publisher

    # Initialise node
    rospy.init_node('topics_sim_to_csv')

    # Set initial velocity
    rospy.set_param("/height_threshold", 0.002)

    rospy.set_param("/feet_in_contact", False)

    publisher = rospy.Publisher('footstep', Bool, queue_size=1)

    odom_sub = message_filters.Subscriber("/aliengo/ground_truth", Odometry)
    cmd_vel_sub = message_filters.Subscriber("/aliengo/wb_controller/joy", Joy)
    fl_trunk_sub = message_filters.Subscriber("/aliengo/wb_controller/lf_foot", CartesianTask)
    fr_trunk_sub = message_filters.Subscriber("/aliengo/wb_controller/rf_foot", CartesianTask)
    rl_trunk_sub = message_filters.Subscriber("/aliengo/wb_controller/lh_foot", CartesianTask)
    rr_trunk_sub = message_filters.Subscriber("/aliengo/wb_controller/rh_foot", CartesianTask)
    footholds_sub = message_filters.Subscriber("/aliengo/wb_controller/contact_forces", ContactForces)

    ts = message_filters.TimeSynchronizer([odom_sub,
                                           cmd_vel_sub,
                                           fl_trunk_sub,
                                           fr_trunk_sub,
                                           rl_trunk_sub,
                                           rr_trunk_sub,
                                           footholds_sub], 10)
    ts.registerCallback(live_extraction)

    rospy.spin()

# Execute main
if __name__ == '__main__':
    main()