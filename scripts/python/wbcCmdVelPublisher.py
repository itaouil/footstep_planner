#!/usr/bin/env python
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
import random

import rospy
import numpy as np
from sensor_msgs.msg import Joy
import dynamic_reconfigure.client

# Global variables
client = None
robot_name = "spot"
MAX_NON_FWD_VELOCITY = 0.55
ACCELERATION_ENABLED = False


def stop(velocity_publisher):
    start_time = rospy.Time.now()
    seconds_to_wait = rospy.Duration(1)
    end_time = start_time + seconds_to_wait

    # Joy message
    joy = Joy()
    joy.header.stamp = rospy.Time.now()
    joy.header.frame_id = "/dev/input/js0"
    joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    rate = rospy.Rate(1000)
    while rospy.Time.now() < end_time:
        # Update joy timestamp
        # joy.header.stamp = rospy.Time.now()

        # Publish
        velocity_publisher.publish(joy)
        rate.sleep()


def stomping(velocity_publisher):
    start_time = rospy.Time.now()
    seconds_to_wait = rospy.Duration(2)
    end_time = start_time + seconds_to_wait

    # Joy message
    joy = Joy()
    joy.header.stamp = rospy.Time.now()
    joy.header.frame_id = "/dev/input/js0"
    joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]

    rate = rospy.Rate(1000)
    while rospy.Time.now() < end_time:
        # Update joy timestamp
        joy.header.stamp = rospy.Time.now()

        # Publish
        velocity_publisher.publish(joy)
        rate.sleep()


def publish_joy_accelerations(joy, velocity_publisher, curr_velocity, motion):
    global client
    global MAX_NON_FWD_VELOCITY

    for next_velocity in np.arange(0.2, 1.0, 0.1):
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

        for _ in range(20):
            # Reset velocity to initial one
            if motion in ["forward", "backward"]:
                joy.axes[1] = curr_velocity
            elif motion in ["right", "left"]:
                joy.axes[0] = curr_velocity
            else:
                joy.axes[3] = curr_velocity

            start_time = rospy.Time.now()
            seconds_to_wait = rospy.Duration.from_sec(random.uniform(1.2, 1.5))
            end_time = start_time + seconds_to_wait

            # Apply current velocity
            rate = rospy.Rate(1000)
            while rospy.Time.now() < end_time or not rospy.get_param("/feet_in_contact"):
                # Update joy timestamp
                joy.header.stamp = rospy.Time.now()

                velocity_publisher.publish(joy)
                rate.sleep()

            start_time = rospy.Time.now()
            seconds_to_wait = rospy.Duration.from_sec(random.uniform(1.2, 1.5))
            end_time = start_time + seconds_to_wait

            # Change velocity to next velocity
            if motion in ["forward", "backward"]:
                joy.axes[1] = next_velocity if motion == "forward" else -next_velocity
            elif motion in ["right", "left"]:
                joy.axes[0] = next_velocity if motion == "left" else -next_velocity
            else:
                joy.axes[3] = next_velocity if motion == "counter" else -next_velocity

            # Apply next velocity
            rate = rospy.Rate(1000)
            while rospy.Time.now() < end_time or not rospy.get_param("/feet_in_contact"):
                # Update joy timestamp
                joy.header.stamp = rospy.Time.now()

                velocity_publisher.publish(joy)
                rate.sleep()


def publish_joy_continuous(joy, velocity_publisher):
    start_time = rospy.Time.now()
    seconds_to_wait = rospy.Duration(30)
    end_time = start_time + seconds_to_wait

    rate = rospy.Rate(1000)
    while rospy.Time.now() < end_time:
        # Update joy timestamp
        joy.header.stamp = rospy.Time.now()

        # Publish
        velocity_publisher.publish(joy)
        rate.sleep()


def joy_publisher():
    global client

    # Initialise node
    rospy.init_node('wbc_cmd_vel_publisher')

    # Publisher
    velocity_publisher = rospy.Publisher(f'/{robot_name}/wolf_controller/joy', Joy, queue_size=1)

    # Start solver
    client = dynamic_reconfigure.client.Client(f"/{robot_name}/wolf_controller")

    rospy.sleep(2)

    while not rospy.is_shutdown():
        for velocity in np.arange(0.1, 1.0, 0.1):
            print(velocity)

            # 1s stomping to avoid instability
            stomping(velocity_publisher)

            # Forward walking
            joy = Joy()
            joy.header.stamp = rospy.Time.now()
            joy.header.frame_id = "/dev/input/js0"
            joy.axes = [0.0, velocity, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
            if ACCELERATION_ENABLED:
                print("Publishing acceleration forward command")
                publish_joy_accelerations(joy, velocity_publisher, velocity, "forward")
            else:
                print("Publishing continuous forward command")
                publish_joy_continuous(joy, velocity_publisher)

            # 1s stomping to avoid instability
            stomping(velocity_publisher)

            # Limit side and rotational velocities to 0.5
            if not ACCELERATION_ENABLED and velocity > MAX_NON_FWD_VELOCITY:
                continue

            # # Clockwise rotation
            # joy = Joy()
            # joy.header.stamp = rospy.Time.now()
            # joy.header.frame_id = "/dev/input/js0"
            # joy.axes = [0.0, 0.0, 0.0, -velocity, 0.0, 0.0, 0.0, 0.0]
            # joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
            # if ACCELERATION_ENABLED:
            #     print("Publishing acceleration clockwise command")
            #     publish_joy_accelerations(joy, velocity_publisher, -velocity, "clockwise")
            # else:
            #     print("Publishing continuous clockwise command")
            #     publish_joy_continuous(joy, velocity_publisher)
            #
            # # 1s stomping to avoid instability
            # stomping(velocity_publisher)
            #
            # # Counterclockwise rotation
            # joy = Joy()
            # joy.header.stamp = rospy.Time.now()
            # joy.header.frame_id = "/dev/input/js0"
            # joy.axes = [0.0, 0.0, 0.0, velocity, 0.0, 0.0, 0.0, 0.0]
            # joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
            # if ACCELERATION_ENABLED:
            #     print("Publishing acceleration counterclockwise command")
            #     publish_joy_accelerations(joy, velocity_publisher, velocity, "counter")
            # else:
            #     print("Publishing continuous counterclockwise command")
            #     publish_joy_continuous(joy, velocity_publisher)
            #
            # # 1s stomping to avoid instability
            # stomping(velocity_publisher)
            #
            # # Right stepping
            # joy = Joy()
            # joy.header.stamp = rospy.Time.now()
            # joy.header.frame_id = "/dev/input/js0"
            # joy.axes = [-velocity, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
            # if ACCELERATION_ENABLED:
            #     print("Publishing acceleration right command")
            #     publish_joy_accelerations(joy, velocity_publisher, -velocity, "right")
            # else:
            #     print("Publishing continuous right command")
            #     publish_joy_continuous(joy, velocity_publisher)
            #
            # # 1s stomping to avoid instability
            # stomping(velocity_publisher)
            #
            # # Left stepping
            # joy = Joy()
            # joy.header.stamp = rospy.Time.now()
            # joy.header.frame_id = "/dev/input/js0"
            # joy.axes = [velocity, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
            # if ACCELERATION_ENABLED:
            #     print("Publishing acceleration left command")
            #     publish_joy_accelerations(joy, velocity_publisher, velocity, "left")
            # else:
            #     print("Publishing continuous left command")
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
