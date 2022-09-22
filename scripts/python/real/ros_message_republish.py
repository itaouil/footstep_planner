#!/usr/bin/python2
import rospy
import rostopic
import argparse
import threading

global g_msg
g_msg = None

def argument_parse():
  parser = argparse.ArgumentParser(description='Republish a topic at a specific rate.')
  parser.add_argument('in_topic_name', type=str, help='Input topic')
  parser.add_argument('rate', type=str, help='Republishing rate (Hz)')
  parser.add_argument('-o', dest='out_topic_name', type=str, default=None, help='Out topic')
  return parser.parse_args()

def msg_callback(msg):
  global g_msg
  g_msg = msg

def publisher():
  global g_msg
  info_flag = True
  while not rospy.is_shutdown():
    if g_msg is not None:
      if info_flag:
        rospy.loginfo("Publish to topic {} at rate {} Hz".format(args.out_topic_name, args.rate))
        info_flag = False
      pub.publish(g_msg)
      rate.sleep()

args = argument_parse()
if args.out_topic_name is None:
  args.out_topic_name = args.in_topic_name + "_resampled"

rospy.init_node('message_republish', anonymous=True)
rate = rospy.Rate(float(args.rate))
topic_class, _, _ = rostopic.get_topic_class(args.in_topic_name)
pub = rospy.Publisher(args.out_topic_name, topic_class, queue_size=1)
rospy.Subscriber(args.in_topic_name, topic_class, msg_callback,
    queue_size=1,  buff_size=2**24)
t = threading.Thread(target=publisher)
t.start()
rospy.loginfo("Waiting for the first message ...")
rospy.spin()
