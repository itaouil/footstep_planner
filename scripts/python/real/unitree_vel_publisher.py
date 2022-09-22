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
from pickle import TRUE
import time
from tkinter import S
import rospy
import random
import numpy as np
from pynput import keyboard
from geometry_msgs.msg import TwistStamped

# Global variables
listener = None
SECONDS_TO_WAIT = 30
STOP_MOTION = False
MAX_NON_FWD_VELOCITY = 1.05
ACCELERATION_ENABLED = True

def on_press(key):
    global STOP_MOTION

    try:
        if key.char == 's':
            STOP_MOTION = True
    except AttributeError:
        pass

def on_release(key):
    return True

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


def publish_joy_accelerations(twist, velocity_publisher, curr_velocity, motion):
    global STOP_MOTION
    global MAX_NON_FWD_VELOCITY

    for next_velocity in np.arange(0.1, 0.9, 0.1):
        # Limit clockwise, counterclockwise, left and right motions to 0.5
        if motion not in ["forward", "backward"] and (
                curr_velocity > MAX_NON_FWD_VELOCITY or next_velocity > MAX_NON_FWD_VELOCITY):
            continue

        # Skip if next velocity to be applied is
        # the same as the current velocity or below
        if abs(next_velocity) - abs(curr_velocity) < 0.05:
            continue

        print("Applying curr velocity: ", curr_velocity, ", with next velocity: ",
              -next_velocity if curr_velocity < 0 else next_velocity)

        for _ in range(25):
            # Reset velocity to initial one
            if motion in ["forward", "backward"]:
                twist.twist.linear.x = curr_velocity
            elif motion in ["right", "left"]:
                twist.twist.linear.y = curr_velocity
            else:
                twist.twist.angular.z = curr_velocity

            # Apply current velocity
            rate = rospy.Rate(1000)
            end_time = time.time() + random.uniform(0.3, 0.6)
            while time.time() < end_time or not rospy.get_param("/feet_in_contact"):
                if STOP_MOTION:
                    input("Press any key to resume")
                    STOP_MOTION = False

                # Update joy timestamp
                twist.header.stamp = rospy.Time.now()

                velocity_publisher.publish(twist)
                rate.sleep()

            # Change velocity to next velocity
            if motion in ["forward", "backward"]:
                twist.twist.linear.x = next_velocity if motion == "forward" else -next_velocity
            elif motion in ["right", "left"]:
                twist.twist.linear.y = next_velocity if motion == "left" else -next_velocity
            else:
                twist.twist.angular.z = next_velocity if motion == "counter" else -next_velocity

            # Apply current velocity
            rate = rospy.Rate(1000)
            end_time = time.time() + random.uniform(0.3, 0.6)
            while time.time() < end_time or not rospy.get_param("/feet_in_contact"):
                if STOP_MOTION:
                    input("Press any key to resume")
                    STOP_MOTION = False
                
                # Update joy timestamp
                twist.header.stamp = rospy.Time.now()

                velocity_publisher.publish(twist)
                rate.sleep()


def publish_joy_continuous(twist, velocity_publisher):
    global STOP_MOTION

    rate = rospy.Rate(1000)
    end_time = time.time() + SECONDS_TO_WAIT

    while time.time() < end_time:
        if STOP_MOTION:
            input("Press any key to resume")
            STOP_MOTION = False

        twist.header.stamp = rospy.Time.now()
        velocity_publisher.publish(twist)
        rate.sleep()


def joy_publisher():
    global listener

    rospy.init_node('unitree_vel_publisher')
    velocity_publisher = rospy.Publisher('/aliengo_bridge/twist_cmd', TwistStamped, queue_size=1)

    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

    while not rospy.is_shutdown():
        for velocity in np.arange(0.0, 0.9, 0.1):

            print(velocity)

            # Forward walking
            twist = get_twist_message([0, 0, 0, velocity, 0, 0])
            if ACCELERATION_ENABLED:
                print("Publishing acceleration forward command")
                publish_joy_accelerations(twist, velocity_publisher, velocity, "forward")
            else:
                if velocity == 0.0:
                    continue
            
                print("Publishing continuous forward command")
                publish_joy_continuous(twist, velocity_publisher)
            
            stomping(velocity_publisher)

            # # Clockwise rotation
            # twist = get_twist_message([0, 0, -velocity, 0, 0, 0])
            # if ACCELERATION_ENABLED:
            #     print("Publishing acceleration clockwise command")
            #     publish_joy_accelerations(twist, velocity_publisher, -velocity, "clockwise")
            # else:
            #     if velocity == 0.0:
            #         continue
            #
            #     print("Publishing continuous clockwise command")
            #     publish_joy_continuous(twist, velocity_publisher)
            #
            # stomping(velocity_publisher)
            #
            # # Counterclockwise rotation
            # twist = get_twist_message([0, 0, velocity, 0, 0, 0])
            # if ACCELERATION_ENABLED:
            #     print("Publishing acceleration counterclockwise command")
            #     publish_joy_accelerations(twist, velocity_publisher, velocity, "counter")
            # else:
            #     if velocity == 0.0:
            #         print("Here: ", velocity)
            #         continue
            #
            #     print("Publishing continuous counterclockwise command")
            #     publish_joy_continuous(twist, velocity_publisher)
            #
            # stomping(velocity_publisher)
            
            #
            # # Right stepping
            # joy = Joy()
            # joy.header.stamp = rospy.Time.now()
            # joy.header.frame_id = "/dev/input/js0"
            # joy.axes = [-velocity, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
            # joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
            # if ACCELERATION_ENABLED:
            #     print("Publishing acceleration right command")
            #     publish_joy_accelerations(joy, velocity_publisher, -velocity, "right")
            # else:
            #     print("Publishing continuous right command")
            #     publish_joy_continuous(joy, velocity_publisher)
            #
            # # Left stepping
            # joy = Joy()
            # joy.header.stamp = rospy.Time.now()
            # joy.header.frame_id = "/dev/input/js0"
            # joy.axes = [velocity, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
            # joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
            # if ACCELERATION_ENABLED:
            #     print("Publishing acceleration left command")
            #     publish_joy_accelerations(joy, velocity_publisher, velocity, "left")
            # else:
            #     print("Publishing continuous left command")
            #     publish_joy_continuous(joy, velocity_publisher)
            #

            # # Forward + clockwise
            # for velocity2 in np.arange(0.1, 0.4, 0.1):
            #     # Counterclockwise rotation
            #     joy = Joy()
            #     joy.header.stamp = rospy.Time.now()
            #     joy.header.frame_id = "/dev/input/js0"
            #     joy.axes = [0.0, velocity, 1.0, -velocity2, 0.0, 1.0, 0.0, 0.0]
            #     joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
            #     print("Publishing forward + clockwise command")
            #     publish_joy_continuous(joy, velocity_publisher)
                
            # # Forward + counter clockwise
            # for velocity2 in np.arange(0.1, 0.6, 0.1):
            #     # Counterclockwise rotation
            #     joy = Joy()
            #     joy.header.stamp = rospy.Time.now()
            #     joy.header.frame_id = "/dev/input/js0"
            #     joy.axes = [0.0, velocity, 1.0, -velocity, 0.0, 1.0, 0.0, 0.0]
            #     joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
            #     print("Publishing forward + counter clockwise command")
            #     publish_joy_continuous(joy, velocity_publisher)
                
            print(f"Sequence {velocity} finished")

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
