#! /usr/bin/env python

import rospy
from std_msgs.msg import Bool

def callback(msg, args):
	print(args);
	print(msg.data)

rospy.init_node('subscriber_example')
rospy.Subscriber("button", Bool, callback, ('some constant :)', 7) ) # args can help you get out of writing multiple similar callbacks
rospy.spin();