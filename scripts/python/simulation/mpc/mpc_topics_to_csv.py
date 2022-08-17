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
import tf
import time
import rospy
import message_filters

# ROS msgs imports
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import TwistStamped

# Global parameters (footstep extraction)
listener = None
max_height = -0.30
fl_max_height = -100
fr_max_height = -100
rl_max_height = -100
rr_max_height = -100
robot_name = "aliengo"
first_footstep = True
footstep_publisher = None
prev_footstep_time = None
lf_height_publisher = None
rf_height_publisher = None
lh_height_publisher = None
rh_height_publisher = None
prev_footstep_flag = False

# Params for feet extraction
lf_rh_moving = True
rf_lh_moving = True

# Global variables
path = f"/home/ilyass/workspace/catkin_ws/src/footstep_planner/data/dataset_sim"
file_object = open(path + "/mpc_motions_accelerations.csv", "a")


def clean_max_heights():
    global fl_max_height
    global fr_max_height
    global rl_max_height
    global rr_max_height

    fl_max_height = -100
    fr_max_height = -100
    rl_max_height = -100
    rr_max_height = -100


def valid_footstep(cmd_msg, heights):
    global footstep_publisher
    global fl_max_height
    global fr_max_height
    global rl_max_height
    global rr_max_height
    global first_footstep
    global prev_footstep_time
    global prev_footstep_flag
    global lf_height_publisher
    global rf_height_publisher
    global lh_height_publisher
    global rh_height_publisher

    # Footstep boolean message
    footstep = Bool()
    footstep.data = False

    if True:
        print("Valid cmd velocity")

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

        print("Logged velocities: ", fl_max_height, fr_max_height, rl_max_height, rr_max_height)

        # Compute feet height difference booleans
        back_height_difference_in_range = abs(rl_height - rr_height) < height_threshold
        front_height_difference_in_range = abs(fl_height - fr_height) < height_threshold

        print("Height differences range: ", front_height_difference_in_range, back_height_difference_in_range)

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
        fl_moving = fl_max_height > max_height
        fr_moving = fr_max_height > max_height
        rl_moving = rl_max_height > max_height
        rr_moving = rr_max_height > max_height

        # Get max heights reached in the motion
        max_heights_sorted = sorted([fl_max_height, fr_max_height, rl_max_height, rr_max_height], reverse=True)
        swing1_max_height = max_heights_sorted[0]
        swing2_max_height = max_heights_sorted[1]

        # Compute booleans for swinging and max height conditions
        swinging_condition = fr_moving != fl_moving and rl_moving != rr_moving and fr_moving == rl_moving and fl_moving == rr_moving
        max_heights_condition = swing1_max_height > max_height and swing2_max_height > max_height

        if not swinging_condition or not max_heights_condition:
            footstep.data = False
            if not max_heights_condition:
                print("Invalid max height condition")
                print(swing1_max_height, swing2_max_height)
            if not swinging_condition:
                print("Invalid swinging condition")
                clean_max_heights()
                print(fl_moving, fr_moving, rl_moving, rr_moving, fl_max_height, fr_max_height, rl_max_height, rr_max_height)

            if not max_heights_condition or not swinging_condition:
                print("Invalid detected footsteps")

        # Clean max height variable if footstep detected
        clean_max_heights()

    # Publish footstep detection boolean
    footstep_publisher.publish(footstep)

    rospy.set_param("/feet_in_contact", footstep.data)

    return footstep.data, fl_moving, fr_moving, rl_moving, rr_moving


def live_extraction(cmd_msg,
                    odom_msg):
    global listener
    global file_object
    
    # Get translation of each foot w.r.t to CoM
    trans_lf, trans_rf, trans_lh, trans_rh = None, None, None, None
    try:
        (trans_lf, _) = listener.lookupTransform('/base_link', '/lf_foot', rospy.Time(0))
        (trans_rf, _) = listener.lookupTransform('/base_link', '/rf_foot', rospy.Time(0))
        (trans_lh, _) = listener.lookupTransform('/base_link', '/lh_foot', rospy.Time(0))
        (trans_rh, _) = listener.lookupTransform('/base_link', '/rh_foot', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("Could not get transforms")
        return
    
    lf_height_publisher.publish(trans_lf[2])
    rf_height_publisher.publish(trans_rf[2])
    lh_height_publisher.publish(trans_lh[2])
    rh_height_publisher.publish(trans_rh[2])

    # Command velocity given
    linear_x = cmd_msg.twist.linear.x
    linear_y = cmd_msg.twist.linear.y
    angular_yaw = cmd_msg.twist.angular.z
    
    # Check at this time a valid footstep is detected
    is_valid_footstep, fl_moving, fr_moving, rl_moving, rr_moving = valid_footstep([linear_x, linear_y, angular_yaw], 
                                                                                   [trans_lf[2], trans_rf[2], trans_lh[2], trans_rh[2]])

    lf_rh_moving = fl_moving and rr_moving
    rf_lh_moving = fr_moving and rl_moving

    # If not a valid footstep, skip callback
    if not is_valid_footstep or lf_rh_moving == rf_lh_moving:
        return
    
    linear_x = cmd_msg.twist.linear.x
    linear_y = cmd_msg.twist.linear.y
    angular_yaw = cmd_msg.twist.angular.z

    file_object.write(str(time.time()) + "," + # 0

                      str(linear_x) + "," + # 1
                      str(linear_y) + "," + # 2
                      str(angular_yaw) + "," + # 3

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
    global listener
    global footstep_publisher
    global lf_height_publisher
    global rf_height_publisher
    global lh_height_publisher
    global rh_height_publisher

    rospy.init_node('topics_sim_to_csv')
    
    listener = tf.TransformListener()
    rospy.set_param("/feet_in_contact", False)
    rospy.set_param("/height_threshold", 0.02)
    footstep_publisher = rospy.Publisher('footstep', Bool, queue_size=1)
    lf_height_publisher = rospy.Publisher('lf_height', Float32, queue_size=1)
    rf_height_publisher = rospy.Publisher('rf_height', Float32, queue_size=1)
    lh_height_publisher = rospy.Publisher('lh_height', Float32, queue_size=1)
    rh_height_publisher = rospy.Publisher('rh_height', Float32, queue_size=1)

    cmd_msg = message_filters.Subscriber(f"/cmd_vel", TwistStamped)
    odom_msg = message_filters.Subscriber(f"/{robot_name}/ground_truth", Odometry)

    ts = message_filters.TimeSynchronizer([cmd_msg,
                                           odom_msg], 10)
    ts.registerCallback(live_extraction)

    rospy.spin()

# Execute main
if __name__ == '__main__':
    main()