#!/usr/bin/env python
from tf.transformations import *
import rospy
import tf2_ros
import numpy as np
import tf2_geometry_msgs
from nav_msgs.msg import Odometry

t265_odom_msg = None

def vicon_odom_callback(msg):
    br = tf.TransformBroadcaster()
    
    # Base rotation
    q_rot = np.array([msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w])
    q_correction = tf.transformations.quaternion_from_euler(-1.5708, 0, 1.5708)
    q_rot = tf.transformations.quaternion_multiply(q_correction, q_rot)
    base_R = tf.transformations.quaternion_matrix(q_rot)
    
    #TODO: adjust translation
    
    # Base translation
    base_T = tf.transformations.translation_matrix([msg.pose.pose.position.x,
                                                    msg.pose.pose.position.y,
                                                    msg.pose.pose.position.z])

    # Publish transform
    br.sendTransform(tf.transformations.translation_from_matrix(base_T),
                     tf.transformations.quaternion_from_matrix(base_R),
                     rospy.Time.now(),
                     "base",
                     "world")
   

def main():
    # Create ROS node
    rospy.init_node('tf_sensors')

    # Subscribe to t265 odometry
    rospy.Subscriber('/vicon/Jumpaolo/odom', Odometry, vicon_odom_callback)

    # Spin it
    rospy.spin()


if __name__ == "__main__":
    main()

