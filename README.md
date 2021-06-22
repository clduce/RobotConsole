## Summary
Robot Console is a web app that brings the power of the modern browser to ROS. There are joysticks, buttons, sliders, check-boxes, gauges,  indicator lights, a video feed, and many other "widgets" that allow the user to remotely control their ROS robot.

## Installation
Before installing Robot Console, make sure you can install ROS and NodeJS.
For Raspberry PI, a recommended image that already has ROS installed can be found here: https://downloads.ubiquityrobotics.com/pi.html
To install run these commands:

    git clone https://github.com/MarkSkinner92/RobotConsole.git
    
    #move your RobotConsole into your ROS directory
    mv RobotConsole ~/catkin_ws/src/pathtoyourrospackage/src

Once it is in your ROS workspace, cd into the RobotConsole and install the dependancies:

    npm i
You might have to do some fiddling around to get all the dependencies to install. These are the dependancies (you shouldn't have to manualy install them, but the list is here just in case )

 - express
 - rosnodejs
 - opencv4nodejs
 - tree-kill
 - @geckos.io/server
 - socket.io

## Troubleshooting
**Cameras**
Some cameras work, and some don't. If a usb camera doesn't work, there's not much that can be done. If the server cannot start because of a camera detection issue, consider hard coding the /dev/video path into the array found at the top of server.js. You can get the list of video paths with this command `v4l2-ctl --list-devices`

**Laggy video**
RobotConsole needs a strong network connection to function. A laggy video is most often a sign of a poor network connection. If the network situation can't be improved, try changing the config settings to a low quality around 20-40, and either 320 x 240 or 640 x 480. the bandwidth used by each frame grows exponentially as the quality increases. 
