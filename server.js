#! /usr/bin/env node

var express = require('express');
const rosnodejs = require('rosnodejs');
//const raspberryPiCamera = require('raspberry-pi-camera-native');
const cv = require('opencv4nodejs');
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
var camArray = [];
var camindex = 0;
app.use(express.static(__dirname + '/public'));
console.log("server running on port "+ PORT);
console.log("looking for cameras...");
let index = 0;

for(let i = 0; i < 15; i++){
	try{
		camArray[index] = new cv.VideoCapture(i);
		camArray[index].set(cv.CAP_PROP_FRAME_WIDTH,320);
		camArray[index].set(cv.CAP_PROP_FRAME_HEIGHT,240);
		//camArray[index].set(cv.CAP_PROP_BUFFER_SIZE,3);
		index++;
		console.log('camera found at index '+i);
	}
	catch{
	}
}
console.log('total of ' + camArray.length + ' cameras found');

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
				
				let latch;
				if(widgets[i].latching) latch = widgets[i].latching;
				else latch = false;
				switch(widgets[i].type){
					case '_button':
					case '_checkbox':
						rospublishers[topic] = nh.advertise(topic, 'std_msgs/Bool',{latching:latch});
					break;
					case '_joystick':
						rospublishers[topic] = nh.advertise(topic, 'geometry_msgs/Vector3');
					break;
					case '_slider':
						rospublishers[topic] = nh.advertise(topic, 'std_msgs/Float64');
					break;
					case '_inputbox':
						let msgType = widgets[i]['msgType'];
						if(msgType == undefined) msgType = 'std_msgs/String';
						if(rospublishers[topic]) rospublishers[topic].shutdown();
						rospublishers[topic] = nh.advertise(topic, msgType);
					break;
					case '_value':
						if(widgets[i]['msgType'] == undefined) widgets[i]['msgType'] = 'std_msgs/String';
						if(rossubscribers[topic]) rossubscribers[topic].shutdown();
						rossubscribers[topic] = nh.subscribe(topic, widgets[i]['msgType'], (msg) => {
							io.emit('telem',{id:i,msg:msg});
						});
					break;
					case '_light':
						if(rossubscribers[topic]) rossubscribers[topic].shutdown();
						rossubscribers[topic] = nh.subscribe(topic, 'std_msgs/Bool', (msg) => {
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
    socket.emit('makeThumbs',camArray.length);
    console.log('current settings on server: ' + JSON.stringify(settingsObject));
  });

  //widgets client to server
  socket.on('WCTS', function(data){
    try{
      settingsObject['widgets'] = data;
      fs.writeFileSync(SETTINGS_PATH, JSON.stringify(settingsObject));
		console.log('recieved updated widget settings from client');
		joinRosTopics();
    }
    catch(e){
      console.log(e);
    }
  });
  //config settings client to server
  socket.on('configSettings', function(data){
    try{
      settingsObject['config'] = data;
      fs.writeFileSync(SETTINGS_PATH, JSON.stringify(settingsObject));
	  console.log('recieved updated config settings from client');
	  //do stuff with your new settings here (camera resolution, running cmds etc.)
    }
    catch(e){
      console.log(e);
    }
  });
  socket.on('pingS', function(data){
    socket.emit('pingR',data);
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
		case '_inputbox':
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

//send thumbs (small camera previews)
let thumbindex = 0;
let getThumb = function(){
	try{
		let frame = camArray[thumbindex].read();
		let image = cv.imencode('.jpg',frame).toString('base64');
		io.emit('thumb',{img:image,index:thumbindex});
	}
	catch{
		console.log('error sending main cam');
	}
	thumbindex++;
	if(thumbindex == camArray.length) thumbindex = 0;
}
setTimeout(getThumb,0);

//send main camera stream
let FPS = 1000/25;
let fc = 0;
let getCam = function(){
	//send main camera stream
	try{
		let frame = camArray[camindex].read();
		let image = cv.imencode('.jpg',frame).toString('base64');
		io.emit('image',image);
	}
	catch{
		console.log('error sending main cam');
	}

	if(fc == 20) fc = 0;
	setTimeout(getCam,FPS);
}
setTimeout(getCam,0);


////READ CAM STREAM FROM PICAM
 //let opts = {//bring these in from config?
	 //width:300,
	 //height:300,
	 //fps:10,
	 //encoding: 'JPEG',
	 //quality:10
 //};
//raspberryPiCamera.on('frame', (frameData) => {
	//if(cameraReciever) cameraReciever.emit('image',frameData.toString('base64'));
//});
//// start capture
//raspberryPiCamera.start(opts);
