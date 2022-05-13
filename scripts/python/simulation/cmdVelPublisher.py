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
from geometry_msgs.msg import Twist


def publishCmdVelAccelerations(msg, pub, curr_velocity):
    for next_velocity in np.arange(0.1, 0.8, 0.1):
        # Skip if next velocity to be applied is
        # the same as the current velocity of below
        if abs(next_velocity) - abs(curr_velocity) < 0.05:
            continue

        next_velocity_correct_direction = next_velocity * -1 if curr_velocity < 0 else next_velocity
        print("Applying curr velocity: ", curr_velocity, ", with next velocity: ", next_velocity_correct_direction)

        for x in range(20):
            startTime = rospy.Time.now()
            #secondsToWait = rospy.Duration(2)
            secondsToWait = rospy.Duration.from_sec(1.5)
            endTime = startTime + secondsToWait

            # Apply current velocity
            rate = rospy.Rate(100)
            while rospy.Time.now() < endTime:
                pub.publish(msg)
                rate.sleep()

            startTime = rospy.Time.now()
            #secondsToWait = rospy.Duration(2)
            secondsToWait = rospy.Duration.from_sec(1.5)
            endTime = startTime + secondsToWait

            # Apply superior velocity (acceleration)
            new_msg = Twist()
            new_msg.linear.x = msg.linear.x if msg.linear.x == 0 else next_velocity_correct_direction
            new_msg.linear.y = msg.linear.y if msg.linear.y == 0 else next_velocity_correct_direction
            new_msg.linear.z = msg.linear.z if msg.linear.z == 0 else next_velocity_correct_direction
            new_msg.angular.x = msg.angular.x if msg.angular.x == 0 else next_velocity_correct_direction
            new_msg.angular.y = msg.angular.y if msg.angular.y == 0 else next_velocity_correct_direction
            new_msg.angular.z = msg.angular.z if msg.angular.z == 0 else next_velocity_correct_direction
            rate = rospy.Rate(100)
            while rospy.Time.now() < endTime:
                pub.publish(new_msg)
                rate.sleep()

    rospy.sleep(1)


def publishCmdVelStutter(msg, pub):
    for x in range(120):
        startTime = rospy.Time.now()
        secondsToWait = rospy.Duration.from_sec(random.uniform(0.5, 1))
        #secondsToWait = rospy.Duration(1)
        endTime = startTime + secondsToWait

        rate = rospy.Rate(100)
        while rospy.Time.now() < endTime:
            pub.publish(msg)
            rate.sleep()

        rospy.sleep(1)


def publishCmdVelContinuous(msg, pub):
    startTime = rospy.Time.now()
    secondsToWait = rospy.Duration(600)
    endTime = startTime + secondsToWait

    rate = rospy.Rate(100)
    while rospy.Time.now() < endTime:
        pub.publish(msg)
        rate.sleep()

    rospy.sleep(1)


def cmdVelPublisher():
    # Initialise node
    rospy.init_node('cmd_vel_publisher')

    # Publisher
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    print("Started publisher")
    rospy.sleep(1)

    while not rospy.is_shutdown():
        for velocity in np.arange(0.1, 0.8, 0.1):
            # Forward walking
            cmd = Twist()
            cmd.linear.x = velocity
            cmd.linear.y = 0
            cmd.linear.z = 0
            cmd.angular.x = 0
            cmd.angular.y = 0
            cmd.angular.z = 0
            # print("Publishing stutter forward command")
            # publishCmdVelStutter(cmd, pub)
            # print("Publishing acceleration forward command")
            # publishCmdVelAccelerations(cmd, pub, velocity)
            print("Publishing continuous forward command")
            publishCmdVelContinuous(cmd, pub)

            # Clockwise rotation
            cmd.linear.x = 0
            cmd.linear.y = 0
            cmd.linear.z = 0
            cmd.angular.x = 0
            cmd.angular.y = 0
            cmd.angular.z = -velocity
            # print("Publishing stutter clockwise command")
            # publishCmdVelStutter(cmd, pub)
            # print("Publishing acceleration clockwise command")
            # publishCmdVelAccelerations(cmd, pub, -velocity)
            print("Publishing continuous clockwise command")
            publishCmdVelContinuous(cmd, pub)

            # Counterclockwise rotation
            cmd.linear.x = 0
            cmd.linear.y = 0
            cmd.linear.z = 0
            cmd.angular.x = 0
            cmd.angular.y = 0
            cmd.angular.z = velocity
            # print("Publishing stutter counterclockwise command")
            # publishCmdVelStutter(cmd, pub)
            # print("Publishing acceleration counterclockwise command")
            # publishCmdVelAccelerations(cmd, pub, velocity)
            print("Publishing continuous counterclockwise command")
            publishCmdVelContinuous(cmd, pub)

            # Right stepping
            cmd.linear.x = 0
            cmd.linear.y = -velocity
            cmd.linear.z = 0
            cmd.angular.x = 0
            cmd.angular.y = 0
            cmd.angular.z = 0
            # print("Publishing stutter right command")
            # publishCmdVelStutter(cmd, pub)
            # print("Publishing acceleration right command")
            # publishCmdVelAccelerations(cmd, pub, -velocity)
            print("Publishing continuous right command")
            publishCmdVelContinuous(cmd, pub)

            # Left stepping
            cmd.linear.x = 0
            cmd.linear.y = velocity
            cmd.linear.z = 0
            cmd.angular.x = 0
            cmd.angular.y = 0
            cmd.angular.z = 0
            # print("Publishing stutter left command")
            # publishCmdVelStutter(cmd, pub)
            # print("Publishing acceleration left command")
            # publishCmdVelAccelerations(cmd, pub, velocity)
            print("Publishing continuous left command")
            publishCmdVelContinuous(cmd, pub)

            print(f"Sequence {velocity} finished")
        break


# Execute main
if __name__ == '__main__':
    try:
        cmdVelPublisher()
    except rospy.ROSInterruptException:
        pass
