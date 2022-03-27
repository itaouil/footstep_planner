#!/usr/bin/env python2.7
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
import tf
import time
import rospy
import message_filters

# ROS msgs imports
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

# Global flags (footstep extraction)
first_footstep = True
prev_footstep_time = None
prev_footstep_flag = False

# Global parameters (footstep extraction)
listener = None
publisher = None
fl_max_height = -100
fr_max_height = -100
rl_max_height = -100
rr_max_height = -100
MAX_HEIGHT = 0.045

# Global variables
robot_name = "aliengo"
path = "/home/itaouil/workspace/code/thesis_ws/src/footstep_planner/data/dataset4_wbc/live_extraction/" + robot_name + "/step_0.065"
file_object = open(path + "/lol_1m.csv", "a")


def clean_max_heights():
    global fl_max_height
    global fr_max_height
    global rl_max_height
    global rr_max_height

    fl_max_height = -100
    fr_max_height = -100
    rl_max_height = -100
    rr_max_height = -100


def valid_footstep(cmd_vel_msg, heights):
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
        fl_height = heights[0]
        fr_height = heights[1]
        rl_height = heights[2]
        rr_height = heights[3]

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
                    # print("Time: ", time.time() - prev_footstep_time)
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
        fl_moving = fl_max_height > MAX_HEIGHT
        fr_moving = fr_max_height > MAX_HEIGHT
        rl_moving = rl_max_height > MAX_HEIGHT
        rr_moving = rr_max_height > MAX_HEIGHT

        # Get max heights reached in the motion
        max_heights_sorted = sorted([fl_max_height, fr_max_height, rl_max_height, rr_max_height], reverse=True)
        swing1_max_height = max_heights_sorted[0]
        swing2_max_height = max_heights_sorted[1]

        # Compute booleans for swinging and max height conditions
        swinging_condition = fr_moving != fl_moving and rl_moving != rr_moving and fr_moving == rl_moving and fl_moving == rr_moving
        max_heights_condition = swing1_max_height > MAX_HEIGHT and swing2_max_height > MAX_HEIGHT

        if not swinging_condition or not max_heights_condition:
            footstep.data = False
            # if not max_heights_condition:
            #     print("Invalid max height condition")
            #     print(swing1_max_height, swing2_max_height)
            # if not swinging_condition:
            #     print("Invalid swinging condition")
            #     print(fl_moving, fr_moving, rl_moving, rr_moving, fl_max_height, fr_max_height, rl_max_height, rr_max_height)

            if not max_heights_condition or not swinging_condition:
                print("Invalid detected footsteps")

        # Clean max height variable if footstep detected
        clean_max_heights()

    # Publish footstep detection boolean
    publisher.publish(footstep)

    rospy.set_param("/feet_in_contact", footstep.data)

    return footstep.data, fl_moving, fr_moving, rl_moving, rr_moving


def live_extraction(odom_msg,
                    cmd_vel_msg):
    # Globals
    global listener
    global file_object

    # Get translation of each foot w.r.t to CoM
    trans_lf, trans_rf, trans_lh, trans_rh = None, None, None, None
    try:
        (trans_lf, _) = listener.lookupTransform('/world', '/lf_foot', rospy.Time(0))
        (trans_rf, _) = listener.lookupTransform('/world', '/rf_foot', rospy.Time(0))
        (trans_lh, _) = listener.lookupTransform('/world', '/lh_foot', rospy.Time(0))
        (trans_rh, _) = listener.lookupTransform('/world', '/rh_foot', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("Could not get transforms")
        return

    # Check at this time a valid footstep is detected
    is_valid_footstep, fl_moving, fr_moving, rl_moving, rr_moving = valid_footstep(cmd_vel_msg, [trans_lf[2], trans_rf[2], trans_lh[2], trans_rh[2]])

    # If not a valid footstep, skip callback
    if not is_valid_footstep:
        return

    # if fl_moving and rr_moving:
    #     print("FL/RR moved")
    # elif fr_moving and rl_moving:
    #     print("FR/RL moved")
    # else:
    #     print("Something is wrong")

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

    file_object.write(str(time.time()) + "," +  # 0

                      str(linear_x) + "," +  # 1
                      str(linear_y) + "," +  # 2
                      str(angular_yaw) + "," +  # 3

                      str(trans_lf[0]) + "," +  # 4
                      str(trans_lf[1]) + "," +  # 5
                      str(trans_lf[2]) + "," +  # 6
                      str(trans_rf[0]) + "," +  # 7
                      str(trans_rf[1]) + "," +  # 8
                      str(trans_rf[2]) + "," +  # 9
                      str(trans_lh[0]) + "," +  # 10
                      str(trans_lh[1]) + "," +  # 11
                      str(trans_lh[2]) + "," +  # 12
                      str(trans_rh[0]) + "," +  # 13
                      str(trans_rh[1]) + "," +  # 14
                      str(trans_rh[2]) + "," +  # 15

                      [[ 2.35069799e-02]
                       [-2.23739261e-03]
                       [ 4.39037271e-05]]

                      str(odom_msg.pose.pose.position.x) + "," +  # 16
                      str(odom_msg.pose.pose.position.y) + "," +  # 17
                      str(odom_msg.pose.pose.position.z) + "," +  # 18

                      str(odom_msg.pose.pose.orientation.x) + "," +  # 19
                      str(odom_msg.pose.pose.orientation.y) + "," +  # 20
                      str(odom_msg.pose.pose.orientation.z) + "," +  # 21
                      str(odom_msg.pose.pose.orientation.w) + "," +   # 22

                      str(odom_msg.twist.twist.linear.x) + "," +  # 23
                      str(odom_msg.twist.twist.linear.y) + "," +  # 24
                      str(odom_msg.twist.twist.linear.z) + "," +  # 25

                      str(odom_msg.twist.twist.angular.x) + "," +  # 26
                      str(odom_msg.twist.twist.angular.y) + "," +  # 27
                      str(odom_msg.twist.twist.angular.z) + "," +  # 28

                      str(fl_moving) + "," +  # 29
                      str(fr_moving) + "," +  # 30
                      str(rl_moving) + "," +  # 31
                      str(rr_moving) + "\n")  # 32


def main():
    # Globals
    global listener
    global publisher

    # Initialise node
    rospy.init_node('topics_sim_to_csv')

    # Set initial velocity
    rospy.set_param("/height_threshold", 0.002)
    rospy.set_param("/feet_in_contact", False)

    listener = tf.TransformListener()
    publisher = rospy.Publisher('footstep', Bool, queue_size=1)

    odom_sub = message_filters.Subscriber("/" + robot_name + "/ground_truth", Odometry)
    cmd_vel_sub = message_filters.Subscriber("/" + robot_name + "/wolf_controller/joy", Joy)

    ts = message_filters.TimeSynchronizer([odom_sub, cmd_vel_sub], 1)
    ts.registerCallback(live_extraction)

    rospy.spin()


# Execute main
if __name__ == '__main__':
    main()