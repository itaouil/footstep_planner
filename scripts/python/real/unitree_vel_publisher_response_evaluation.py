#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    Publishes velocity commands to the /cmd_vel
    topic in order to make the robot perform
    the following motions:

        * Forward
        * Clockwise
        * Counterclockwise
        * Right
        * Left
"""

# General imports
import time
import rospy
import random
import numpy as np
from pynput import keyboard
from geometry_msgs.msg import TwistStamped

# Global variables
listener = None
STOP_MOTION = False
ADDITIONAL_TIME = 0
SECONDS_TO_WAIT = 30
MAX_NON_FWD_VELOCITY = 1.05
ACCELERATION_ENABLED = False


def on_press(key):
    global STOP_MOTION

    try:
        if key.char == 's':
            STOP_MOTION = not STOP_MOTION
            print("STOP MOTION: ", STOP_MOTION)
    except AttributeError:
        pass


def on_release(key):
    return True


def send_zero_velocity(velocity_publisher):
    global ADDITIONAL_TIME

    twist = TwistStamped()
    twist.twist.angular.x = 0
    twist.twist.angular.y = 0
    twist.twist.angular.z = 0
    twist.twist.linear.x = 0
    twist.twist.linear.y = 0
    twist.twist.linear.z = 0

    start = time.time()
    rate = rospy.Rate(1000)
    while STOP_MOTION:
        twist.header.stamp = rospy.Time.now()
        velocity_publisher.publish(twist)
        rate.sleep()
    
    ADDITIONAL_TIME = time.time() - start


def get_twist_message(velocities):
    twist = TwistStamped()
    twist.header.stamp = rospy.Time.now()
    twist.twist.angular.x = velocities[0]
    twist.twist.angular.y = velocities[1]
    twist.twist.angular.z = velocities[2]
    twist.twist.linear.x = velocities[3]
    twist.twist.linear.y = velocities[4]
    twist.twist.linear.z = velocities[5]
    return twist


def stop(velocity_publisher):
    twist = get_twist_message([0, 0, 0, 0, 0, 0])
    velocity_publisher.publish(twist)


def stomping(velocity_publisher):
    start_time = rospy.Time.now()
    seconds_to_wait = rospy.Duration(2)
    end_time = start_time + seconds_to_wait
    twist = get_twist_message([0, 0, 0, 0.001, 0, 0])

    rate = rospy.Rate(1000)
    while rospy.Time.now() < end_time:
        twist.header.stamp = rospy.Time.now()
        velocity_publisher.publish(twist)
        rate.sleep()

def joy_publisher():
    global listener

    rospy.init_node('unitree_vel_publisher')
    velocity_publisher = rospy.Publisher('/aliengo_bridge/twist_cmd', TwistStamped, queue_size=1)

    rate = rospy.Rate(1000)

    while not rospy.is_shutdown():
        twist = get_twist_message([0, 0, 0, 0.1, 0, 0])
        end_time = time.time() + 0.2
        while time.time() < end_time:
            twist.header.stamp = rospy.Time.now()
            velocity_publisher.publish(twist)
            rate.sleep()

        twist = get_twist_message([0, 0, 0, 0.9, 0, 0])
        end_time = time.time() + 0.5
        while time.time() < end_time:
            twist.header.stamp = rospy.Time.now()
            velocity_publisher.publish(twist)
            rate.sleep()

        twist = get_twist_message([0, 0, 0, 0.2, 0, 0])
        end_time = time.time() + 0.3
        while time.time() < end_time:
            twist.header.stamp = rospy.Time.now()
            velocity_publisher.publish(twist)
            rate.sleep()

        twist = get_twist_message([0, 0, 0, 0.1, 0, 0])
        end_time = time.time() + 0.4
        while time.time() < end_time:
            twist.header.stamp = rospy.Time.now()
            velocity_publisher.publish(twist)
            rate.sleep()

        twist = get_twist_message([0, 0, 0, 0.9, 0, 0])
        end_time = time.time() + 0.2
        while time.time() < end_time:
            twist.header.stamp = rospy.Time.now()
            velocity_publisher.publish(twist)
            rate.sleep()

        break

    # Stop robot fully
    stomping(velocity_publisher)
    stop(velocity_publisher)


# Execute main
if __name__ == '__main__':
    try:
        joy_publisher()
    except rospy.ROSInterruptException:
        pass
