#! /usr/bin/env node

var express = require('express');
const rosnodejs = require('rosnodejs');
const raspberryPiCamera = require('raspberry-pi-camera-native');
const fs = require('fs');
var app = express();
var PORT = 3000;
const SETTINGS_PATH = __dirname + '/settings.json';
var server = app.listen(PORT);
var settingsObject;
var rosready = false;
var nh, rospublishers={}, rossubscribers={};
var cameraReciever;//socket id of the client displaying the camera stream
var socketsOpen = 0;
app.use(express.static(__dirname + '/public'));
console.log("server running on port 80 and " + PORT)

var socket = require('socket.io');
var io = socket(server);


function joinRosTopics(){
	fs.readFile(SETTINGS_PATH, (err, data) => {
		if (err) throw err;
		settingsObject = JSON.parse(data);
   
		let widgets = settingsObject['widgets'];
		
		//attatch ROS publishers and listeners
		for(let i = 0; i < widgets.length; i++){
			let topic = widgets[i].topic;
			if(topic != '' && topic != '/'){
				console.log(widgets[i].type + ' connecting to   '+widgets[i].topic);
				switch(widgets[i].type){
					case '_button':
					case '_checkbox':
						rospublishers[topic] = nh.advertise(topic, 'std_msgs/Bool');
					break;
					case '_joystick':
						rospublishers[topic] = nh.advertise(topic, 'geometry_msgs/Vector3');
					break;
					case '_slider':
						rospublishers[topic] = nh.advertise(topic, 'std_msgs/Float64');
					break;
					case '_value':
						if(widgets[i]['msgType'] == undefined) widgets[i]['msgType'] = 'std_msgs/String';
						if(rossubscribers[topic]) rossubscribers[topic].shutdown();
						rossubscribers[topic] = nh.subscribe(topic, widgets[i]['msgType'], (msg) => {
							io.emit('telem',{id:i,msg:msg});
						});
					break;
					case '_gauge':
						if(widgets[i]['msgType'] == undefined) widgets[i]['msgType'] = 'std_msgs/Float64';
						if(rossubscribers[topic]) rossubscribers[topic].shutdown();
						rossubscribers[topic] = nh.subscribe(topic, widgets[i]['msgType'], (msg) => {
							io.emit('telem',{id:i,msg:msg});
						});
					break;
				}
			}
		}
		rosready = true;
	});
}
  
rosnodejs.initNode('/webserver').then(() => {
	nh = rosnodejs.nh;
	joinRosTopics();
}).catch((e) => {
	console.log('Error connecting to ROS: ' + e);
});


io.sockets.on('connection', function(socket){
	socketsOpen++;
	io.emit('instanceCount',socketsOpen);
  console.log('made connection');
  //get settings from json and send to client
  fs.readFile(SETTINGS_PATH, (err, data) => {
    if (err) throw err;
    settingsObject = JSON.parse(data);
    socket.emit('settings',settingsObject);
    console.log('current settings on server: ' + JSON.stringify(settingsObject));
  });

  //widgets client to server
  socket.on('WCTS', function(data){
    try{
      settingsObject['widgets'] = data;
      fs.writeFileSync(SETTINGS_PATH, JSON.stringify(settingsObject));
		console.log('recieved updated settings from client');
		joinRosTopics();
    }
    catch(e){
      console.log(e);
    }
  });

  //ROS client to server
  socket.on('ROSCTS', function(data){
    settingsObject['widgets'] = data;
    var topic = data.topic;
    switch(data.type){
		case '_button':
		case '_checkbox':
			rospublishers[topic].publish({ data:data.pressed});
		break;
		case '_slider':
			rospublishers[topic].publish({ data:data.value});
		break;
		case '_joystick':
			rospublishers[topic].publish({ x:data.x,y:data.y,z:0});
		break;
	}
    console.log('Publish Ros ' + JSON.stringify(data));
  });
  //remove all subscribers/publishers from topic
  socket.on('shutROS', function(data){
	  if(rossubscribers[data]) rossubscribers[data].shutdown();
	  if(rospublishers[data]) rospublishers[data].shutdown();
  });
  socket.on('setCam', function(data){
    camindex = data;
    console.log(`Change Camera to ${data}`);
  });
  socket.on('setScreen1', function(data){
    cameraReciever = socket;
  });
  socket.on('disconnect', function(data){
    socketsOpen--;
    io.emit('instanceCount',socketsOpen);
  });
});


//READ CAM STREAM FROM PICAM
 let opts = {//bring these in from config?
	 width:300,
	 height:300,
	 fps:10,
	 encoding: 'JPEG',
	 quality:10
 };
raspberryPiCamera.on('frame', (frameData) => {
	if(cameraReciever) cameraReciever.emit('image',frameData.toString('base64'));
});
// start capture
raspberryPiCamera.start(opts);
