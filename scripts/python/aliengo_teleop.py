#!/usr/bin/env python
# -*- coding: utf-8 -*-
#credits to: https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

from __future__ import print_function

# General imports
import tty
import sys
import select
import termios
import numpy as np

# ROS imports
import rospy
import roslib

# Messages import
from unitree_legged_msgs.msg import HighCmd

class Teleop:
    def __init__(self):
        self.velocity_publisher = rospy.Publisher('/aliengo_bridge/cmd', HighCmd, queue_size = 1)

        self.turn = rospy.get_param("~turn", 0.1)
        self.speed = rospy.get_param("~speed", 0.1)

        self.msg = """
                   Reading from the keyboard  and Publishing to Twist!
                   ---------------------------
                   Moving around:
                   u    i    o
                   j    k    l
                   m    ,    .

                   2: counterclowise rotation
                   1: clockwise rotation

                   t : up (+z)
                   b : down (-z)

                   anything else : stop

                   q/z : increase/decrease max speeds by 10%
                   w/x : increase/decrease only linear speed by 10%
                   e/c : increase/decrease only angular speed by 10%

                   CTRL-C to quit
                   """

        self.velocityBindings = {
                # forward/backward, left/right, clockwise/counter clockwise
                'u': (1, 0, 1),
                'i': (1, 0, 0),
                'o': (1, 0, -1),
                'j': (0, 1, 0),
                'l': (0, -1, 0),
                'm': (-1, 0, -1),
                ',': (-1, 0, 0),
                '.': (-1, 0, 1),
                '1': (0, 0, -1),
                '2': (0, 0, 1)
            }

        self.speedBindings = {
                'q': (1.1, 1.1),
                'z': (.9, .9),
                'w': (1.1, 1),
                'x': (.9, 1),
                'e': (1, 1.1),
                'c': (1, .9),
            }
        
        self.poll_keys()

    def poll_keys(self):
        self.settings = termios.tcgetattr(sys.stdin)

        status = 0
        cmd_attempts = 0

        try:
            print(self.msg)
            print(self.vels( self.speed, self.turn))
            
            while not rospy.is_shutdown():
                key = self.getKey()
                if key in self.velocityBindings.keys():
                    side = self.velocityBindings[key][1]
                    forward = self.velocityBindings[key][0]
                    rotation = self.velocityBindings[key][2]
                    
                    if cmd_attempts > 1:
                        # Populate message
                        highCmd = HighCmd()
                        highCmd.mode = 2 # // 0: idle, default stand 1: forced stand 2: walk continuously
                        highCmd.yaw = 0.0
                        highCmd.roll = 0.0
                        highCmd.pitch = 0.0
                        highCmd.bodyHeight = 0.0
                        highCmd.footRaiseHeight = 0.0
                        highCmd.sideSpeed = side * self.speed
                        highCmd.rotateSpeed = rotation * self.turn
                        highCmd.forwardSpeed = forward * self.speed

                        # Print message to be sent
                        print("Sending command: ", highCmd)

                        # Publish command
                        self.velocity_publisher.publish(highCmd)

                    cmd_attempts += 1
                    
                elif key in self.speedBindings.keys():
                    self.speed = self.speed * self.speedBindings[key][0]
                    self.turn = self.turn * self.speedBindings[key][1]
                    
                    print(self.vels(self.speed, self.turn))

                    if (status == 14):
                        print(self.msg)
                    
                    status = (status + 1) % 15

                else:
                    cmd_attempts = 0
                    if (key == '\x03'):
                        break

        except Exception as e:
            print(e)

        finally:
            # Populate message
            highCmd = HighCmd()
            highCmd.mode = 0 # 0: idle, default stand 1: forced stand 2: walk continuously
            highCmd.yaw = 0
            highCmd.roll = 0
            highCmd.pitch = 0
            highCmd.sideSpeed = 0
            highCmd.bodyHeight = 0
            highCmd.rotateSpeed = 0
            highCmd.forwardSpeed = 0
            highCmd.footRaiseHeight = 0

            # Print message to be sent
            print("Sending command: ", highCmd)

            # Publish command
            self.velocity_publisher.publish(highCmd)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == "__main__":
    rospy.init_node('aliengo_teleop')
    teleop = Teleop()
