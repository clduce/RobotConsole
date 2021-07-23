#! /usr/bin/env python
import rospy
from std_msgs.msg import String

rospy.init_node('publisher_example')

publisher = rospy.Publisher("stringout", String, queue_size = 1)
rate = rospy.Rate(2) #2 hz

while not rospy.is_shutdown():
	msg = String()
	msg.data = 'hello, this is a test'
	publisher.publish(msg)
	rate.sleep()
