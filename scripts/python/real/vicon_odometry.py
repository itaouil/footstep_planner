#!/usr/bin/python

import rospy
import tf.transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

vicon_odometry_pub = None

def vicon_callback(msg):
    global vicon_odometry_pub
    
    # Correct vicon base rotation
    q_vicon_base = np.array([msg.transform.rotation.x,
                             msg.transform.rotation.y,
                             msg.transform.rotation.z,
                             msg.transform.rotation.w])
    q_correction = tf.transformations.quaternion_from_euler(-1.5708, 0, 1.5708)
    q_vicon_base_corrected = tf.transformations.quaternion_multiply(q_correction, q_vicon_base)
    print(q_vicon_base_corrected)

    odometry_msg = Odometry()
    odometry_msg.header.stamp = rospy.Time.now()
    odometry_msg.header.frame_id = "world"
    odometry_msg.child_frame_id = "base"
    odometry_msg.pose.pose.position.x = msg.transform.translation.x
    odometry_msg.pose.pose.position.y = msg.transform.translation.y
    odometry_msg.pose.pose.position.z = msg.transform.translation.z
    odometry_msg.pose.pose.orientation.x = q_vicon_base_corrected[0]
    odometry_msg.pose.pose.orientation.y = q_vicon_base_corrected[1]
    odometry_msg.pose.pose.orientation.z = q_vicon_base_corrected[2]
    odometry_msg.pose.pose.orientation.w = q_vicon_base_corrected[3]

    vicon_odometry_pub.publish(odometry_msg)    

def listener():
    global vicon_odometry_pub

    rospy.init_node('vicon_odometry', anonymous=True)
    
    rospy.Subscriber("/vicon/Jumpaolo/base", TransformStamped, vicon_callback)
    
    vicon_odometry_pub = rospy.Publisher('vicon/Jumpaolo/odom', Odometry, queue_size=1)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
