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
import rospy
import numpy as np
from sensor_msgs.msg import Joy
import dynamic_reconfigure.client

# Dynamic reconfigure client
client = None

# Max non forward velocity
MAX_NON_FWD_VELOCITY=0.55


def stop(velocity_publisher):
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
    joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]

    rate = rospy.Rate(1000)
    while rospy.Time.now() < end_time:
        # Update joy timestamp
        joy.header.stamp = rospy.Time.now()

        # Publish
        velocity_publisher.publish(joy)
        rate.sleep()


def publish_joy_accelerations(joy, velocity_publisher, curr_velocity):
    global client
    global MAX_NON_FWD_VELOCITY

    for next_velocity in np.arange(0.0, 1.0, 0.1):
        # Limit clockwise, counterclockwise, left and right motions to 0.5
        if joy.axes in [[-1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0],
                        [1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]] \
                and (curr_velocity > MAX_NON_FWD_VELOCITY or next_velocity > MAX_NON_FWD_VELOCITY):
            continue

        # Skip if next velocity to be applied is
        # the same as the current velocity of below
        if abs(next_velocity) - abs(curr_velocity) < 0.05:
            continue

        print("Applying curr velocity: ", curr_velocity, ", with next velocity: ", next_velocity)

        for _ in range(48):
            # Reset velocity to initial one
            rospy.set_param("/current_velocity", float(curr_velocity))
            params = {"set_linear_vel": curr_velocity, "set_angular_vel": curr_velocity}
            client.update_configuration(params)

            start_time = rospy.Time.now()
            seconds_to_wait = rospy.Duration.from_sec(1.25)
            end_time = start_time + seconds_to_wait

            # Apply current velocity
            rate = rospy.Rate(1000)
            while rospy.Time.now() < end_time:
                # Update joy timestamp
                joy.header.stamp = rospy.Time.now()

                velocity_publisher.publish(joy)
                rate.sleep()

            start_time = rospy.Time.now()
            seconds_to_wait = rospy.Duration.from_sec(1.25)
            end_time = start_time + seconds_to_wait

            # Change velocity
            rospy.set_param("/current_velocity", float(next_velocity))
            params = {"set_linear_vel": next_velocity, "set_angular_vel": next_velocity}
            client.update_configuration(params)

            rate = rospy.Rate(1000)
            while rospy.Time.now() < end_time:
                # Update joy timestamp
                joy.header.stamp = rospy.Time.now()

                velocity_publisher.publish(joy)
                rate.sleep()

    # Reset velocity to initial one
    rospy.set_param("/current_velocity", float(curr_velocity))
    params = {"set_linear_vel": curr_velocity, "set_angular_vel": curr_velocity}
    client.update_configuration(params)


def publish_joy_continuous(joy, velocity_publisher):
    start_time = rospy.Time.now()
    seconds_to_wait = rospy.Duration(300)
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
    velocity_publisher = rospy.Publisher('/aliengo/wb_controller/joy', Joy, queue_size=10)

    # Set footstep height to 0.15 and start solver
    client = dynamic_reconfigure.client.Client("/aliengo/wb_controller")
    params = {"set_step_height": 0.15, "activate_solver": True}
    client.update_configuration(params)

    # Sleep for 1 second to allow solver to activate
    rospy.sleep(1)

    # Reset feet positions
    stomping(velocity_publisher)

    while not rospy.is_shutdown():
        for velocity in np.arange(0.0, 1.0, 0.1):
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
            #print("Publishing acceleration forward command")
            #publish_joy_accelerations(joy, velocity_publisher, velocity)
            print("Publishing c/home/ilyass/workspace/code/thesis_ws/srcontinuous forward command")
            publish_joy_continuous(joy, velocity_publisher)

            # 1s stomping to avoid instability
            stomping(velocity_publisher)

            # Clockwise rotation
            joy = Joy()
            joy.header.stamp = rospy.Time.now()
            joy.header.frame_id = "/dev/input/js0"
            joy.axes = [-1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0]
            joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
            # print("Publishing acceleration clockwise command")
            # publish_joy_accelerations(joy, velocity_publisher, velocity)
            # print("Publishing continuous clockwise command")
            # publish_joy_continuous(joy, velocity_publisher)

            # 1s stomping to avoid instability
            stomping(velocity_publisher)

            # Counterclockwise rotation
            joy = Joy()
            joy.header.stamp = rospy.Time.now()
            joy.header.frame_id = "/dev/input/js0"
            joy.axes = [1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
            joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
            # print("Publishing acceleration counterclockwise command")
            # publish_joy_accelerations(joy, velocity_publisher, velocity)
            # print("Publishing continuous counterclockwise command")
            # publish_joy_continuous(joy, velocity_publisher)

            # 1s stomping to avoid instability
            stomping(velocity_publisher)

            # Right stepping
            joy = Joy()
            joy.header.stamp = rospy.Time.now()
            joy.header.frame_id = "/dev/input/js0"
            joy.axes = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
            # print("Publishing acceleration right command")
            # publish_joy_accelerations(joy, velocity_publisher, velocity)
            # print("Publishing continuous right command")
            # publish_joy_continuous(joy, velocity_publisher)

            # 1s stomping to avoid instability
            stomping(velocity_publisher)

            # Left stepping
            joy = Joy()
            joy.header.stamp = rospy.Time.now()
            joy.header.frame_id = "/dev/input/js0"
            joy.axes = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
            # print("Publishing acceleration left command")
            # publish_joy_accelerations(joy, velocity_publisher, velocity)
            # print("Publishing continuous left command")
            # publish_joy_continuous(joy, velocity_publisher)

            #print(f"Sequence {velocity} finished")
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
