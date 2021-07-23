#! /usr/bin/env python

# ================ WARNING! PLEASE READ ======================

# If you are using this at the same time as the UI server,
# Make sure the UI server hasn't already opened the camera
# you can do this by adding the desired video path to the "video_blacklist" in "hardcoded_settings.json"
# You can also disable video in "hardcoded_settings.json"
# Restart the server to apply changes

path = '/dev/video1';

import rospy
import cv2
from sensor_msgs.msg import CompressedImage

rospy.init_node('camerasender')

publisher = rospy.Publisher("imgout", CompressedImage, queue_size = 1)
rate = rospy.Rate(30) #30 hz

print('publishing /dev/video1 to topic "imgout" at 30 fps');

#1. Open a camera
cam = cv2.VideoCapture(path)

#2. Set width and height small, so it doesn't overload the socket and cause high ping
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 180)


while not rospy.is_shutdown():
	#3. Read a frame from the camera
	ret, frame = cam.read()
	
	#4. Encode frame to jpeg buffer (with a lower quality)
	success, imbuf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
	
	#5. Convert image buffer to byte array
	imbytes = imbuf.tobytes()
	
	#6. Construct image message with byte array and publish
	msg = CompressedImage()
	msg.header.stamp = rospy.Time.now()
	msg.format = "jpeg"
	msg.data = imbytes
	publisher.publish(msg)
	rate.sleep()
