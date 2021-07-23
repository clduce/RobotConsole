//Provides help messages for widgets and other things

function showHelp(){
	document.getElementById('configWindow').style.left = '48%';
	document.getElementById('helpWindow').style.display = 'inline';
}
function hideHelp(){
	document.getElementById('configWindow').style.left = '25%';
	document.getElementById('helpWindow').style.display = 'none';
}
function toggleHelp(){
	if(document.getElementById('helpWindow').style.display == 'none'){
		showHelp();
	}
	else{
		hideHelp();
	}
}

const generalMsg = {
	'latch':'Latching can be enabled to ensure late subscribers get the message.'
}

const helpMessages = {
	'_button':toParagraph('Once you have entered a ROS topic name, you can choose the values to be sent on button press and release. If the type is std_msgs/Bool, only false and False will be interpreted as false. Anything else will be interpreted as true'),
	'_joystick':toParagraph('Sends a geometry_msgs/Vector3 message where X is the horizontal joystic value from -1 to 1, and Y is the vertical joystick value from -1 to 1. the Z component is not used. A message is only sent when the joystick moves.'),
	'_checkbox':toParagraph("Once you have entered a ROS topic name, you can choose the values to be sent on button press and release. Like the button, if the type is std_msgs/Bool, only false and False will be interpreted as false. Anything else will be interpreted as true. " +generalMsg.latch+' If the initial state is checked, the checkbox will start out checked.'),
	'_slider':toParagraph('If orient vertical is checked, the slider will apear vertically with the lowest value on the bottom. '+generalMsg.latch+' Flip direction will make left/down be the max instead. The default value should be a natural number within the min and max. The repeat delay is the ammount of time in ms when a key or gamepad must be held before moving the slider again.'),
	'_inputbox':toParagraph('Very usefull for testing.'),
	'_dropdown':toParagraph("Sends it's new value when the dropdown menu changes "+generalMsg.latch+' The uppermost option will be the default. Remove option removes from the bottom up. Add another option adds to the bottom.'),
	'_value':toParagraph('Prefix text is put right in front of the incoming value, and the postfix text is appended to the end. No space is inserted arround the raw value.'),
	'_indicator':toParagraph('subscribes to std_msgs/Bool.'),
	'_gauge':toParagraph('A bold tick with a value written by it appears every "big tick interval", and the area between big ticks are devided into "Subdevisions" ticks.'),
	'_arm':toParagraph('If the segment is using a fixed angle, put the desired angle next to it. If the segment is using an array element, put an array index beside it. Arms are added and removed from the bottom.'),
	'_logger':toParagraph(generalMsg.latch+'. This widget stores its data.'),
	'_audio':toParagraph('This widget plays the sound of the recieved integer index'),
	'_box':toParagraph('When a widget is placed into the panel, it becomes a child of the panel, moves with it, and gets deleted if the panel is deleted. when a panel is moved under a widget, it does nothing.'),
	'_serial':(toParagraph('ROS to USB is the topic name used to send data from your ROS node across the network and out the serial port. Any string you publish to this topic will be concatenated with the ROS to USB line ending and sent as chars out the serial port.')+toParagraph('You subscribe to the USB to ROS topic to get outbound data from the serial port. this data is stored in a buffer untill a USB to ROS split with char is found. The char/chars will be stripped and the resulting string will be published on that topic.')),
	'_mic':(toParagraph('You can play this audio on the robot by launching play.launch from the audio_play package. On the terminal it looks like this: roslaunch audio_play play.launch. The node that is launched with play.launch only subscribes to /audio/audioplay of type audio_common_msgs/AudioData.msg so for this to work the topic must be set accordingly.')+toParagraph('The audio being played is mono with a sample rate of 16000')),
	'_speaker':(toParagraph('You can launch the capture.launch file from the audio_capture package to publish microphone data to this widget. Its topic is always /audio/audiocapture. From this node, incoming audio data is mono with a sample rate of 16000')+toParagraph('You can publish any WAVE audio (pcm) to this topic and the browser will play it if the widget is unmuted.')),
	'_mouse':(toParagraph('When you click on the white box, your mouse will be locked. You will need to hit escape to stop your mouse from being locked.')+toParagraph('This widget publishes a vector3 message where x and y are the mouse velocities, z is the mouse button. 0 being no buttons, 1 being the left mouse button, and 2 being the right mouse button.')+toParagraph('On the robot, you can keep track of a the mouse position by adding the velocities to a stored position vector every time a new message is recieved.')),
}

//returns string as pagargraph element
function toParagraph(s){
	return `<p>${s}</p>`;
}

function updateHelpWindow(type){
	console.log(type, helpMessages[type]);
	document.getElementById('helpArea').innerHTML = helpMessages[type] || '';
}