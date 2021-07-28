#! /usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import CompressedImage

rospy.init_node('imagesenderpng')

publisher = rospy.Publisher("imgout", CompressedImage, queue_size = 1)
rate = rospy.Rate(1) #1 hz
print('publishing /home/ubuntu/testpng.png to topic "imgout" once per second');

#1. read the image into opencv
im = cv2.imread("/home/ubuntu/testpng.png", cv2.IMREAD_UNCHANGED); #cv2.IMREAD_UNCHANGED is included to also load the alpha channel

#2. encode the image into a jpeg buffer
success,imbuf = cv2.imencode(".png", im)

#3. convert the image buffer into bytes
imbytes = imbuf.tobytes()

while not rospy.is_shutdown():
	msg = CompressedImage()
	msg.header.stamp = rospy.Time.now()
	msg.format = "png"
	msg.data = imbytes
	publisher.publish(msg)
	rate.sleep()
