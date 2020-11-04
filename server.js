#! /usr/bin/env node
const SETTINGS_PATH = __dirname + '/settings.json';
const HARDCODED_SETTINGS_PATH = __dirname + '/hardcoded_settings.json';

var express = require('express');
const rosnodejs = require('rosnodejs');
//const raspberryPiCamera = require('raspberry-pi-camera-native');
const cv = require('opencv4nodejs');
const cp = require('child_process');
var kill = require('tree-kill');
const fs = require('fs');
var app = express();
var PORT = 3000;
var server = app.listen(PORT);
var settingsObject,hardcoded;
var rosready = false;
var nh, rospublishers={}, rossubscribers={};
//var cameraReciever;//socket id of the client displaying the camera stream
var socketsOpen = 0;
var camArray = [],camJSON={"presets": [{"width":"320","height":"240","quality":100,"name":"low res"}],"camsettings":[{"preset":0,"name":"pi cam"}]};
var camindex = 0;
app.use(express.static(__dirname + '/public'));
console.log("server running on port "+ PORT);
console.log("looking for cameras...");
let index = 0;
let cmds = {};

for(let i = 0; i < 10; i++){
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
var io = socket(server, {pingInterval: 100});
io.set('origins','*:*');


function joinRosTopics(){
	fs.readFile(SETTINGS_PATH, (err, data) => {
		if (err) throw err;
		let settingsObject;
		try{
			settingsObject = JSON.parse(data);
		}catch(e){console.log(e);}
		if(settingsObject){
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
							rospublishers[topic] = nh.advertise(topic, 'std_msgs/Float64',{latching:latch});
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
								io.emit('telem',{topic:topic,id:i,msg:msg});
							});
						break;
						case '_light':
							if(rossubscribers[topic]) rossubscribers[topic].shutdown();
							rossubscribers[topic] = nh.subscribe(topic, 'std_msgs/Bool', (msg) => {
								io.emit('telem',{topic:topic,id:i,msg:msg});
							});
						break;
						case '_audio':
							if(rossubscribers[topic]) rossubscribers[topic].shutdown();
							rossubscribers[topic] = nh.subscribe(topic, 'std_msgs/Int16', (msg) => {
								io.emit('telem',{topic:topic,id:i,msg:msg});
							});
						break;
						case '_gauge':
							if(widgets[i]['msgType'] == undefined) widgets[i]['msgType'] = 'std_msgs/Float64';
							if(rossubscribers[topic]) rossubscribers[topic].shutdown();
							rossubscribers[topic] = nh.subscribe(topic, widgets[i]['msgType'], (msg) => {
								io.emit('telem',{topic:topic,id:i,msg:msg});
							});
						break;
					}
				}
			}
			rosready = true;
		}
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
    camJSON = settingsObject.config.cams;
    
    //set initial resolutions for cameras
    for(let i = 0; i < Math.min(camArray.length,camJSON.camsettings.length); i++){
		camArray[i].set(cv.CAP_PROP_FRAME_WIDTH,parseInt(camSettings(camJSON,i).width));
		camArray[i].set(cv.CAP_PROP_FRAME_HEIGHT,parseInt(camSettings(camJSON,i).height));
	}
    
    socket.emit('settings',settingsObject);
    socket.emit('makeThumbs',camArray.length);
    console.log('current settings on server: ' + JSON.stringify(settingsObject));
  });
  //get settings from json and send to client
  fs.readFile(HARDCODED_SETTINGS_PATH, (err, data) => {
    if (err) throw err;
    hardcoded = JSON.parse(data);
    console.log(JSON.stringify(hardcoded));
    socket.emit('hardcoded_settings',hardcoded);
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
	  camJSON = data.cams;
	  for(let i = 0; i < camArray.length; i++){
		camArray[i].set(cv.CAP_PROP_FRAME_WIDTH,parseInt(camSettings(camJSON,i).width));
		camArray[i].set(cv.CAP_PROP_FRAME_HEIGHT,parseInt(camSettings(camJSON,i).height));
	  }
    }
    catch(e){
      console.log(e);
    }
  });
  //start a child process
  socket.on('cmd', function(data){
	  console.log(hardcoded);
	if(hardcoded.show_terminal){
		cmds[data] = cp.spawn(data,[],{shell:true});
		cmds[data].stdout.on('data', stdout => {
			socket.emit('cmdOut',stdout.toString());
			console.log(stdout.toString());
		});
		cmds[data].stderr.on('data', stderr => {
			socket.emit('cmdOut',stderr.toString());
			console.log(stderr.toString());
		});
		cmds[data].on('close', code => {
			socket.emit('cmdOut','exit code: '+code+'\n');
			socket.emit('removeCmd',data);
			console.log(code);
		});
	}else{
		console.log('terminal has been disabled');
	}
  });
  socket.on('stopcmd', function(data){
	  if(hardcoded.show_terminal){
		console.log('stopping '+data);
		kill(cmds[data].pid);
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
  //change resolution of camera. c is camera, v is preset value (index)
  socket.on('setPreset', function(data){
	console.log(data.c,data.v);
	console.log(camJSON.presets[data.v].name);
	if(camArray[data.c]){
		camArray[data.c].set(cv.CAP_PROP_FRAME_WIDTH,parseInt(camJSON.presets[data.v].width));
		camArray[data.c].set(cv.CAP_PROP_FRAME_HEIGHT,parseInt(camJSON.presets[data.v].height));
	}
  });
  //socket.on('setScreen1', function(data){
    //cameraReciever = socket;
  //});
  socket.on('closeOtherSockets', function(data){
    socket.broadcast.emit('closeSocket','');
  });
  socket.on('disconnect', function(data){
	socket.disconnect();
    socketsOpen--;
    io.emit('instanceCount',socketsOpen);
  });
});

//send thumbs (small camera previews)
//let thumbindex = 0;
//let getThumb = function(){
	//try{
		//let frame = camArray[thumbindex].read();
		////let image = cv.imencode('.jpg',frame,[1,20]).toString('base64');
		////io.emit('thumb',{img:image,index:thumbindex});
		//cv.imencodeAsync('.jpg',frame,[1,10]).then(function(result){
			//io.emit('thumb',{img:result.toString('base64'),index:thumbindex});
			//thumbindex++;
			//if(thumbindex == camArray.length) thumbindex = 0;
			//setTimeout(getThumb,200);
		//}).catch(function(e){console.log(e);});
	//}
	//catch{
		//console.log('error sending main cam');
	//};
//}
//setTimeout(getThumb,0);


//cams is the entire cam json from config
//cam index is the camera number in the camArray
function camSettings(cams, camindex){
	return cams.presets[cams.camsettings[camindex].preset];
}

//send main camera stream
let getCam = function(){
	//send main camera streams
	try{
		let qual = 100;
		if(camJSON[camindex]) qual = parseInt(camSettings(camJSON,camindex).quality);
		cv.imencodeAsync('.jpg',camArray[camindex].read(),[1,qual]).then(function(result){
			io.emit('image',result.toString('base64'));
		}).catch(function(e){console.log(e);});
		setTimeout(getCam,30);
	}
	catch{
		console.log('error sending main cam');
	}
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
