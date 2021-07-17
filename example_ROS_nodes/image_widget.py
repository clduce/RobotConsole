#! /usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

rospy.init_node('imagesender')

publisher = rospy.Publisher("imgout", CompressedImage, queue_size = 1)
rate = rospy.Rate(1) #1 hz

im = cv2.imread("/home/ubuntu/background.jpg");

success,imbuf = cv2.imencode(".jpg", im)
imbytes = imbuf.tobytes()

while not rospy.is_shutdown():
	msg = CompressedImage()
	msg.header.stamp = rospy.Time.now()
	msg.format = "jpeg"
	msg.data = imbytes
	publisher.publish(msg)
	rate.sleep()
