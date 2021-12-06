#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Publishes velocity commands to the /cmd_vel
    topic in order to make the robot perform
    the following motions:

        * Forward walking
        * Clockwise rotation
        * Counterclockwise rotation
        * Right stepping
        * Left stepping
        * Diagonal right walking
        * Diagonal left walking
"""

# General imports
import rospy
import random
import numpy as np
from sensor_msgs.msg import Joy
import dynamic_reconfigure.client

# Dynamic reconfigure client
client = None


def stop(pub):
    start_time = rospy.Time.now()
    seconds_to_wait = rospy.Duration(1)
    end_time = start_time + seconds_to_wait

    # Joy message
    joy = Joy()
    joy.header.stamp = rospy.Time.now()
    joy.header.frame_id = "/dev/input/js0"
    joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    rate = rospy.Rate(1000)
    while rospy.Time.now() < end_time:
        # Update joy timestamp
        # joy.header.stamp = rospy.Time.now()

        # Publish
        pub.publish(joy)
        rate.sleep()


def stomping(pub):
    start_time = rospy.Time.now()
    seconds_to_wait = rospy.Duration(1)
    end_time = start_time + seconds_to_wait

    # Joy message
    joy = Joy()
    joy.header.stamp = rospy.Time.now()
    joy.header.frame_id = "/dev/input/js0"
    joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]

    rate = rospy.Rate(1000)
    while rospy.Time.now() < end_time:
        # Update joy timestamp
        joy.header.stamp = rospy.Time.now()

        # Publish
        pub.publish(joy)
        rate.sleep()


def publish_joy_accelerations(joy, pub, curr_velocity):
    global client

    for next_velocity in np.arange(0.1, 0.6, 0.1):
        # Skip if next velocity to be applied is
        # the same as the current velocity of below
        if abs(next_velocity) - abs(curr_velocity) < 0.05:
            continue

        print("Applying curr velocity: ", curr_velocity, ", with next velocity: ", next_velocity)

        for x in range(24):
            # Reset velocity to initial one
            rospy.set_param("/current_velocity", float(curr_velocity))
            params = {"set_linear_vel": curr_velocity, "set_angular_vel": curr_velocity}
            client.update_configuration(params)

            start_time = rospy.Time.now()
            seconds_to_wait = rospy.Duration.from_sec(2.5)
            end_time = start_time + seconds_to_wait

            # Apply current velocity
            rate = rospy.Rate(1000)
            while rospy.Time.now() < end_time:
                # Update joy timestamp
                joy.header.stamp = rospy.Time.now()

                pub.publish(joy)
                rate.sleep()

            start_time = rospy.Time.now()
            seconds_to_wait = rospy.Duration.from_sec(2.5)
            end_time = start_time + seconds_to_wait

            # Change velocity
            rospy.set_param("/current_velocity", float(next_velocity))
            params = {"set_linear_vel": next_velocity, "set_angular_vel": next_velocity}
            client.update_configuration(params)

            rate = rospy.Rate(1000)
            while rospy.Time.now() < end_time:
                # Update joy timestamp
                joy.header.stamp = rospy.Time.now()

                pub.publish(joy)
                rate.sleep()

    # Reset velocity to initial one
    rospy.set_param("/current_velocity", float(curr_velocity))
    params = {"set_linear_vel": curr_velocity, "set_angular_vel": curr_velocity}
    client.update_configuration(params)


def publish_joy_stutter(msg, pub):
    for x in range(120):
        start_time = rospy.Time.now()
        seconds_to_wait = rospy.Duration.from_sec(random.uniform(0.5, 1))
        end_time = start_time + seconds_to_wait

        rate = rospy.Rate(1000)
        while rospy.Time.now() < end_time:
            pub.publish(msg)
            rate.sleep()

        rospy.sleep(1)


def publish_joy_continuous(joy, pub):
    start_time = rospy.Time.now()
    seconds_to_wait = rospy.Duration(120)
    end_time = start_time + seconds_to_wait

    rate = rospy.Rate(1000)
    while rospy.Time.now() < end_time:
        # Update joy timestamp
        joy.header.stamp = rospy.Time.now()

        # Publish
        pub.publish(joy)
        rate.sleep()


def joy_publisher():
    global client

    # Initialise node
    rospy.init_node('wbc_cmd_vel_publisher')

    # Create dynamic reconfigure client
    client = dynamic_reconfigure.client.Client("/aliengo/wb_controller")

    # Set footstep height to 0.15 and start solver
    params = {"set_step_height": 0.15, "activate_solver": True}
    client.update_configuration(params)

    # Publisher
    pub = rospy.Publisher('/aliengo/wb_controller/joy', Joy, queue_size=10)

    print("Started publisher")

    while not rospy.is_shutdown():
        for velocity in np.arange(0.1, 0.6, 0.1):
            print(velocity)

            # Change base and rotation velocity
            params = {"set_linear_vel": velocity, "set_angular_vel": velocity}
            client.update_configuration(params)

            rospy.set_param("/current_velocity", float(velocity))

            # Forward walking
            joy = Joy()
            joy.header.stamp = rospy.Time.now()
            joy.header.frame_id = "/dev/input/js0"
            joy.axes = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
            # print("Publishing stutter forward command")
            # publish_joy_stutter(joy, pub)
            # print("Publishing acceleration forward command")
            # publish_joy_accelerations(joy, pub, velocity)
            print("Publishing continuous forward command")
            publish_joy_continuous(joy, pub)

            # 1s stomping to avoid instability
            stomping(pub)

            # Clockwise rotation
            # joy = Joy()
            # joy.header.stamp = rospy.Time.now()
            # joy.header.frame_id = "/dev/input/js0"
            # joy.axes = [-1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0]
            # joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
            # print("Publishing stutter clockwise command")
            # publish_joy_stutter(joy, pub)
            # print("Publishing acceleration clockwise command")
            # publish_joy_accelerations(joy, pub, velocity)
            # print("Publishing continuous clockwise command")
            # publish_joy_continuous(joy, pub)

            # 1s stomping to avoid instability
            stomping(pub)

            # Counterclockwise rotation
            # joy = Joy()
            # joy.header.stamp = rospy.Time.now()
            # joy.header.frame_id = "/dev/input/js0"
            # joy.axes = [1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
            # joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
            # print("Publishing stutter counterclockwise command")
            # publish_joy_stutter(joy, pub)
            # print("Publishing acceleration counterclockwise command")
            # publish_joy_accelerations(joy, pub, velocity)
            # print("Publishing continuous counterclockwise command")
            # publish_joy_continuous(joy, pub)

            # 1s stomping to avoid instability
            stomping(pub)

            # Right stepping
            joy = Joy()
            joy.header.stamp = rospy.Time.now()
            joy.header.frame_id = "/dev/input/js0"
            joy.axes = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
            # print("Publishing stutter right command")
            # publish_joy_stutter(joy, pub)
            # print("Publishing acceleration right command")
            # publish_joy_accelerations(joy, pub, velocity)
            print("Publishing continuous right command")
            publish_joy_continuous(joy, pub)

            # 1s stomping to avoid instability
            stomping(pub)

            # Left stepping
            joy = Joy()
            joy.header.stamp = rospy.Time.now()
            joy.header.frame_id = "/dev/input/js0"
            joy.axes = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
            # print("Publishing stutter left command")
            # publish_joy_stutter(joy, pub)
            # print("Publishing acceleration left command")
            # publish_joy_accelerations(joy, pub, velocity)
            print("Publishing continuous left command")
            publish_joy_continuous(joy, pub)

            print(f"Sequence {velocity} finished")
        break

    # Stop robot fully
    stomping(pub)
    stop(pub)


# Execute main
if __name__ == '__main__':
    try:
        joy_publisher()
    except rospy.ROSInterruptException:
        pass
