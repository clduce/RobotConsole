## Summary
Robot Console is a web app that brings the power of the modern browser to ROS. There are joysticks, buttons, sliders, check-boxes, gauges,  indicator lights, a video feed, and many other "widgets" that allow the user to remotely control their ROS robot.

## Installation
Before installing Robot Console, make sure you can install ROS and NodeJS.
For Raspberry PI, a recommended image that already has ROS installed can be found here: https://downloads.ubiquityrobotics.com/pi.html
To install run these commands:

    git clone https://github.com/MarkSkinner92/RobotConsole.git
    
    #move your RobotConsole into your catkin workspace directory
    mv RobotConsole ~/catkin_ws/src/pathtoyourrospackage/src

Once it is in your ROS workspace, cd into the RobotConsole and install the dependancies:

    npm i
You might have to do some fiddling around to get all the dependencies to install. These are the dependancies (you shouldn't have to manualy install them, but the list is here just in case )

 - express
 - rosnodejs
 - opencv4nodejs
 - tree-kill
 - socket.io

## Technical Details
**Setting up:**
An express server `server.js` hosts the contents of the `/public` directory, home of all the HTML and JavaScript responsible for running the UI. When the page loads, the client establishes a websocket connection back to the server. The server then sends `settings.json` to the client, and the client uses that JSON to create widgets. `settings.json` contains everything you can adjust from the UI.

**The widgetArray:** `settings.json` contains everything you can adjust from the UI. It has two objects: `config` and `widgets`. Every widget is stored as an object in `widgets`. Here is an example of what a widget object might look like:

    {
        "type":"_checkbox",     <- a widget type always has an underscore in front
        "id":0,     <- this is the actual html id of the widget's div

        "left":"361px", <- the next 8 lines are for position and size,
        "top":"235px",  <- the widgets sometimes have to stick to different
        "useLeft":true, <- sides of the screen
        "useTop":true,
        "right":"687px",
        "bottom":"514px",
        "w":"200px",
        "h":"52px",

        "topic":"mytopic",    <- the ROS topic of this widget
        "useROS":true,        <- whether or not this widget uses ROS
        
        "label":"Label",      <- the rest of theese properties are specific to
        "useGamepad":true,    <- the widget
        "useButton":-1,
        "initial":true,
        "latching":true,
        "textColor":"#000000",
        "useKeys":false,
        "usekey_hotkey":""
    }
Every widget is structured in more or less the same way. The `config` portion contains misc things to do with the camera, background colour, macros, etc.


**Telemetry:**
After the socket connection is started and all the widgets are built, the `server.js` and `main.js` create listeners for telemetry. on the server, its `socket.on('ROSCTS')` (ROS client to server), and on the client, its `socket.on('telem')`. When the server recieves telemetry, it creates a ROS message and publishes it out on a ROS publisher. When the client recieves telemetry, it finds the widget based on it's id and updates it graphically. The ROS publishers and subscribers the server creates are stored in arrays which are shut down and restarted every time a widget is updated. 

## Troubleshooting
**Cameras:**
Some cameras work, and some don't. If the server cannot start because of a camera detection issue, consider hard coding your camera paths or blacklisting them in `hardcoded_settings.json`. You can get the list of video paths with this command `v4l2-ctl --list-devices`

If none of the above worked, try disabling the video in `hardcoded_settings.json` and using a full screen image widget. From here, you can use python or anything that connects to ROS to handle the camera back end on your own. (An example python script can be found in `example_nodes/image_widget`

**Laggy video:**
RobotConsole needs a strong network connection to function. A laggy video is most often a sign of a poor network connection. If the network situation can't be improved, try changing the config settings to a low quality around 20-40, and either 320 x 240 or 640 x 480. the bandwidth used by each frame grows exponentially as the quality increases. 

Other things to try include 

 - closing other clients
 - ensuring other bandwidth-using programs running on the pi are kept to a minimum
- lower the resolution and frame rate of any image widgets in your python code
