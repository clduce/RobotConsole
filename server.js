#! /usr/bin/env node
const SETTINGS_PATH = __dirname + '/settings.json';
const HARDCODED_SETTINGS_PATH = __dirname + '/hardcoded_settings.json';
const RESET_SOCKET_AFTER_MS = 400; //if the ping gets above this, the socket and cameras will reset

var express = require('express');
const rosnodejs = require('rosnodejs');
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
var socketsOpen = 0;
var camArray = [],camJSON={"presets": [{"width":"320","height":"240","quality":100,"name":"low res"}],"camsettings":[{"preset":0,"name":"pi cam"}]};
var camindex = 0;
app.use(express.static(__dirname + '/public'));
console.log("server running on port "+ PORT);
console.log("looking for cameras...");
let index = 0;
let cmds = {};
var cps = [];
var mainQuality = 90;
var hardcodedLoaded = false;

for(let i = 0; i < 10; i++){
	try{
		camArray[index] = new cv.VideoCapture(i);
		camArray[index].set(cv.CAP_PROP_FRAME_WIDTH,320);
		camArray[index].set(cv.CAP_PROP_FRAME_HEIGHT,240);
		cps[index]=0;
		index++;
		console.log('camera found at index '+i);
	}
	catch{
	}
}
console.log('total of ' + camArray.length + ' cameras found');

var socket = require('socket.io');
var io = socket(server, {pingInterval: 100, pingTimeout: RESET_SOCKET_AFTER_MS});
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
						case '_compass':
							if(widgets[i]['msgType'] == undefined) widgets[i]['msgType'] = 'std_msgs/Int16';
							if(rossubscribers[topic]) rossubscribers[topic].shutdown();
							rossubscribers[topic] = nh.subscribe(topic, widgets[i]['msgType'], (msg) => {
								io.emit('telem',{topic:topic,id:i,msg:msg});
								console.log(msg);
							});
						break;
						case '_horizon':
							if(widgets[i]['msgType'] == undefined) widgets[i]['msgType'] = 'std_msgs/Float64Array';
							if(rossubscribers[topic]) rossubscribers[topic].shutdown();
							rossubscribers[topic] = nh.subscribe(topic, widgets[i]['msgType'], (msg) => {
								io.emit('telem',{topic:topic,id:i,msg:msg.data});
							});
						break;
						case '_rosImage':
							rossubscribers[topic] = nh.subscribe(topic, 'sensor_msgs/CompressedImage', (msg) => {
								io.emit('telem',{topic:topic,id:i,msg:msg.data});
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
    io.emit('cmdStopButtons',Object.keys(cmds));
  
  //get settings from json and send to client
  fs.readFile(SETTINGS_PATH, (err, data) => {
    if (err) throw err;
    settingsObject = JSON.parse(data);
    camJSON = settingsObject.config.cams;
    
    //set initial resolutions for cameras
    for(let i = 0; i < Math.min(camArray.length,camJSON.camsettings.length); i++){
		camArray[i].set(cv.CAP_PROP_FRAME_WIDTH,parseInt(camSettings(camJSON,i).width));
		camArray[i].set(cv.CAP_PROP_FRAME_HEIGHT,parseInt(camSettings(camJSON,i).height));
		cps[i] = camJSON.camsettings[i].preset;
	}
    mainQuality = parseInt(camSettings(camJSON,0).quality);
    		console.log('main quality is ' + mainQuality);
    socket.emit('settings',settingsObject);
    socket.emit('makeThumbs',camArray.length,camindex,cps);
    console.log('current settings on server: ' + JSON.stringify(settingsObject));
  });
  //get settings from json and send to client
  fs.readFile(HARDCODED_SETTINGS_PATH, (err, data) => {
    if (err) throw err;
    hardcoded = JSON.parse(data);
    console.log(JSON.stringify(hardcoded));
    socket.emit('hardcoded_settings',hardcoded);
    hardcodedLoaded=true;
  });

  //widgets client to server
  socket.on('WCTS', function(data){
    try{
		if(settingsObject.config.saveWidgets){
			settingsObject['widgets'] = data;
			fs.writeFileSync(SETTINGS_PATH, JSON.stringify(settingsObject));
			console.log('recieved updated widget settings from client');
			joinRosTopics();
		}
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
		cps[i] = camJSON.camsettings[i].preset;
	  }
    }
    catch(e){
      console.log(e);
    }
  });
  //start a child process
  socket.on('cmd', function(data){
	if(hardcodedLoaded && hardcoded.show_terminal){
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
			if(cmds[data]) delete cmds[data];
		});
	}else{
		console.log('terminal has been disabled');
	}
  });
  socket.on('stopcmd', function(data){
	  if(hardcodedLoaded && hardcoded.show_terminal && cmds[data]){
		console.log('stopping '+data);
		kill(cmds[data].pid);
		delete cmds[data];
	  }
	});
  socket.on('exit', function(data){
	  process.exit(data);
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
  //change resolution of camera. c is camera, v is preset value (index)
  socket.on('setPreset', function(data){
	console.log(data.c,data.v);
	console.log(camJSON.presets[data.v].name);
	if(camArray[data.c]){
		camArray[data.c].set(cv.CAP_PROP_FRAME_WIDTH,parseInt(camJSON.presets[data.v].width));
		camArray[data.c].set(cv.CAP_PROP_FRAME_HEIGHT,parseInt(camJSON.presets[data.v].height));
		cps[data.c] = data.v;
		if(data.c == camindex) mainQuality = parseInt(camJSON.presets[data.v].quality);
	}
  });
  socket.on('setCam', function(data){
    camindex = data;
    mainQuality = parseInt(camJSON.presets[cps[data]].quality);
    console.log(`Change Camera to ${data}`);
  });
  socket.on('closeOtherSockets', function(data){
    socket.broadcast.emit('closeSocket','');
  });
  socket.on('disconnect', function(data){
	socket.disconnect();
    socketsOpen--;
    io.emit('instanceCount',socketsOpen);
  });
});


//cams is the entire cam json from config
//cam index is the camera number in the camArray
function camSettings(cams, camindex){
	return cams.presets[cams.camsettings[camindex].preset];
}

let oldtime = 0;
let retrieveCam = function(){
	//time = new Date().getTime();
	camArray[camindex].readAsync().then(function(result){
		//result.rotate(int 0-4), result.flip(int)
		if(!result.empty){
			cv.imencodeAsync('.jpg',result,[cv.IMWRITE_JPEG_QUALITY,mainQuality]).then(function(result){
				io.emit('image',result.toString('base64'));
			}).catch((e)=>{console.log(e)});
		}
		setTimeout(retrieveCam,0);
	}).catch((e)=>{console.log("can't read camera " + e);});
	//console.log(oldtime-time);
	//oldtime = time;
}
setTimeout(retrieveCam,0);
