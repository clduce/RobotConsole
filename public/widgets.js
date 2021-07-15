//parses json into widgets
//contains functionality for all Widgets
//a hashtable like array that stores indexes at different id's
var indexMap, topicMapIndex;
// TODO: add details for all types of widgets
function widgetFromJson(json){
  var type = json['type'];
  if(!type){
	  console.log(json,'widget does not have a type and cannot be created');
	  return;
  }
  console.log('creating widget: '+type);
  var tile = widgetFromId(type);
  if(!tile){
	  console.log(`widget type ${type} doesn't exist in this version of UI`);
	  return;
  }
  tile.id = json['id'];
  tile.style.zIndex = 20;
  if(type == '_box') tile.style.zIndex = 5;
  
  set4style(tile,json);
	
	tile.style.width = json['w'];
	tile.style.height = json['h'];
  
  switch (type) {
    case '_button':
      tile.querySelector('#button_ap').innerText = json['label'];
      if(json['fontsize']) tile.querySelector('#button_ap').style.fontSize = parseFloat(json['fontsize'])+'px';
    break;
    case '_checkbox':
      tile.querySelector('#checkbox_text_ap').innerText = json['label'];
      tile.querySelector('#checkbox_ap').checked = json['initial'];
      tile.querySelector('#checkbox_text_ap').style.color = json['textColor'];
      if(json['latching']) sendToRos(json['topic'],{value:json['initial'] ? json['onPress'] : json['onRelease']},'_checkbox');
    break;
    case '_joystick':
      var canvas = tile.querySelector('#canvas_ap');
      canvas.height = parseInt(json['h'])-20;
      canvas.width = parseInt(json['w']);
      drawJoystick(canvas,0,0);
    break;
    case '_slider':
      tile.querySelector('#slider_ap').min = json['min'];
      tile.querySelector('#slider_ap').max = json['max'];
      tile.querySelector('#slider_ap').value= (parseInt(json['min']) + parseInt(json['max'])
      )/2;
      tile.querySelector('#slider_ap').step = json['step'];
    break;
    case '_value':
		tile.querySelector('#text_ap').innerText='Waiting for ROS...';
		tile.querySelector('#text_ap').style.color = json['textColor'];
	break;
	case '_light':
		tile.querySelector('#text_ap').innerText=json['text'];
	break;
	case '_audio':
		tile.querySelector('#speaker_ap').className = json['hideondrive']?'':'showOnDrive';
	break;
	case '_gauge':
      var canvas = tile.querySelector('#gauge_ap');
      canvas.height = parseInt(json['h'])-20;
      canvas.width = parseInt(json['w']);
      canvas.setAttribute("data-config",JSON.stringify({min:json.min,max:json.max,bigtick:json.bigtick,smalltick:json.smalltick, title:json.label}));
      drawGauge(canvas,json.min,json);
    break;
	case '_compass':
	break;
	case '_horizon':
	break;
	case '_rosImage':
		  let img_ap = tile.querySelector('#img_ap');
		  if(json.src) img_ap.src = json.src;
		  if(json.aspr) img_ap.className = 'showOnDrive containImage';
		  if(json.opac) img_ap.style.opacity = json.opac+'%';
		  if(json.center) centerImageWidget(tile,json);
	break;
	case '_arm':
      var canvas = tile.querySelector('#arm_ap');
      canvas.height = parseInt(json['h'])-20;
      canvas.width = parseInt(json['w']);
	  drawArm(canvas, json.arms);
    break;
	case '_dropdown':
      tile.querySelector('#selector_ap').innerHTML = generateSelectorOptions(json.dropdowns);
    break;
    case '_text':
		tile.querySelector('#text_ap').innerText=json['text'];
		tile.querySelector('#text_ap').style.color = json['textColor'];
	break;
	case '_box':
		tile.querySelector('#panel_ap').style.backgroundColor = json['bkColor'];
	break;
    case '_logger':
    case '_inputbox':
	case  '_serial':
	case  '_mic':
	case  '_speaker':
		  tile.querySelector('#label_ap').innerText = json['label'] || '';
	case  '_panel':
	case '_mouse':
    break;
	default:
		  console.log(`widget type ${type} doesn't exist in this version of UI`);
		  return;
	break;
  }

  initFunctionality(json['type'],tile,tile.id);
}
//json is the widget array for the tile widget
function set4style(tile,json){
	if(!json) json=widgetArray[indexMap[tile.id]];
	if(json['useTop']){
		 tile.style.top = json['top'];
		 tile.style.bottom = '';
	}else{
		tile.style.top = '';
		tile.style.bottom = json['bottom'];
	}
	if(json['useLeft']){
		tile.style.right = '';
		tile.style.left = json['left'];
	}else{
		tile.style.left = '';
		tile.style.right = json['right'];
	}
	if(json.type == '_rosImage' && json.center){
		centerImageWidget(tile,json);
	}
}
function centerImageWidget(tile,json){
	tile.style.left = (window.innerWidth/2 - parseInt(tile.style.width)/2)+'px';
	tile.style.top = (window.innerHeight/2 - parseInt(tile.style.height)/2)+'px';
}
function get4position(tile){
	let index = indexMap[tile.id];
	if(widgetArray[index]['useTop']){
		 widgetArray[index].top = parseInt(tile.style.top) + 'px';
		 widgetArray[index].bottom = (window.innerHeight - parseInt(tile.style.height) - parseInt(tile.style.top)) + 'px';
	}else{
		widgetArray[index].bottom = parseInt(tile.style.bottom) + 'px';
		widgetArray[index].top = (window.innerHeight - parseInt(tile.style.height) - parseInt(tile.style.bottom)) + 'px';
	}
	if(widgetArray[index]['useLeft']){
		 widgetArray[index].left = parseInt(tile.style.left) + 'px';
		 widgetArray[index].right = (window.innerWidth - parseInt(tile.style.width) - parseInt(tile.style.left)) + 'px';
	}else{
		widgetArray[index].right = parseInt(tile.style.right) + 'px';
		widgetArray[index].left = (window.innerWidth - parseInt(tile.style.width) - parseInt(tile.style.right)) + 'px';
	}
}
function useClosest(tile){
	let index = indexMap[tile.id];
	if(parseInt(widgetArray[index].left) < parseInt(widgetArray[index].right)) widgetArray[index].useLeft = true;
	else widgetArray[index].useLeft = false;
	if(parseInt(widgetArray[index].top) < parseInt(widgetArray[index].bottom)) widgetArray[index].useTop = true;
	else widgetArray[index].useTop = false;
}
function useSide(tile,uleft,utop){
	let index = indexMap[tile.id];
	if(uleft) widgetArray[index].useLeft = true;
	else widgetArray[index].useLeft = false;
	if(utop) widgetArray[index].useTop = true;
	else widgetArray[index].useTop = false;
}
//returns json from widget
//assigns the newWidget an id and functionality
function makeUnique(type,newWidget){
  console.log('creating widget clone: ' + type);
  let thisWidget = JSON.parse('{}');
  thisWidget['type'] = type;
  var thisID = generateUniqueId(widgetArray.length);//this will be the index of newWidget because it hasn't been pushed to the widget array yet
  thisWidget['id'] = thisID;
  newWidget.id = thisID;
  newWidget.style.zIndex = 30;
  if(type == '_box') newWidget.style.zIndex = 5;
  thisWidget['left'] = newWidget.style.left;
  thisWidget['top'] = newWidget.style.top;
  thisWidget['useLeft'] = true;
  thisWidget['useTop'] = true;
  thisWidget['right'] = '';
  thisWidget['bottom'] = '';
  thisWidget['w'] = newWidget.style.width;
  thisWidget['h'] = newWidget.style.height;
  thisWidget['topic'] = newWidget.querySelector('#header').childNodes[0].data;
  thisWidget['screen'] = 1;
  thisWidget['useROS'] = true;

  switch (type) {
    case '_button':
      thisWidget['label'] = newWidget.querySelector('#button_ap').innerText;
      thisWidget['useGamepad'] = true;
      thisWidget["useButton"] = -1;
    break;
    case '_checkbox':
      thisWidget['label'] = newWidget.querySelector('#checkbox_text_ap').innerText;
      thisWidget['useGamepad'] = true;
      thisWidget["useButton"] = -1;
    break;
    case '_joystick':
      thisWidget['useGamepad'] = true;
      thisWidget['useKeys'] = false;
      thisWidget["useAxis"] = -1;
      thisWidget["usekey_up"] = "w";
      thisWidget["usekey_left"] = "a";
      thisWidget["usekey_down"] = "s";
      thisWidget["usekey_right"] = "d";
    break;
    case '_value':
		thisWidget['msgType'] = 'std_msgs/String';
	break;
	case '_light':
		thisWidget['msgType'] = 'std_msgs/Bool';
		thisWidget['text'] = 'label';
	break;
	case '_gauge':
      thisWidget['label'] = 'Example';
      thisWidget['min'] = 0;
      thisWidget["max"] = 100;
      thisWidget["bigtick"] = 20;
      thisWidget["smalltick"] = 4;
    break;
	case '_arm':
		thisWidget.arms = [{mode:1,data:60,armlength:5,color:'#000000'},{mode:1,data:-90,armlength:3,color:'#00FF00'}];
	break;
    case '_box':
		thisWidget['useTop'] = true;
		thisWidget['useLeft'] = true;
    case '_text':
		thisWidget['useROS'] = false;
    break;
    case '_logger':
		thisWidget['msgType'] = 'std_msgs/String';
    break;
    default:
	break;
  }
  initFunctionality(type,newWidget,thisID);
  return thisWidget;
}
//##### This function only runs if a case for the widget type is also declared in widgetFromJson()
function initFunctionality(type, newWidget,thisID){
  var jsw = widgetArray[indexMap[thisID]];
  switch(type){
    case '_button':
      //setup brodcast functionality for element
      newWidget.querySelector('#button_ap').onmousedown = function(){
        sendToRos(jsw['topic'],{value:jsw['onPress'] || true},jsw['type']);
      };
      newWidget.querySelector('#button_ap').onmouseup = function(){
        sendToRos(jsw['topic'],{value:jsw['onRelease'] || false},jsw['type']);
      };
    break;
    case '_checkbox':
      //setup brodcast functionality for element
      newWidget.querySelector('#checkbox_ap').onchange = function(e){
        sendToRos(jsw['topic'],{value:e.target.checked ? jsw['onPress'] : jsw['onRelease']},jsw['type']);
      };
    break;
    case '_slider':
      //setup brodcast functionality for element
       if(jsw){
		   if(jsw['vertical']){
				newWidget.querySelector('#slider_ap').className += ' vertical';
				newWidget.querySelector('#slider_ap').style.width =(parseInt(jsw['h'])-27) + 'px';
			}
			newWidget.querySelector('#slider_ap').value = parseFloat(setSliderDirection(jsw['default'],jsw));
			sendToRos(jsw['topic'],{value:parseFloat(jsw['default'])},jsw['type']);
			newWidget.querySelector('#slider_ap').oninput = function(e){
				sendToRos(jsw['topic'],{
					value:setSliderDirection(e.target.value,jsw)
				},jsw['type']);
			};
		}
    break;
    case '_inputbox':
      //setup brodcast functionality for element
	  function send(){
          sendToRos(jsw['topic'],{value:newWidget.querySelector('#input_ap').value},jsw['type']);
		  newWidget.querySelector('#input_ap').value = '';
	  }
      newWidget.querySelector('#inputboxbutton').onmousedown = function(e){
        send();
      };
	  newWidget.querySelector('#input_ap').onkeyup = function(e){
		if(e.key == 'Enter'){
        	send();
		}
      };
    break;
	case '_mouse':
		var mouseCanvas = newWidget.querySelector('#mousecanvas_ap');
		mouseCanvas.writeText = function(text,text2){
			let ctx = mouseCanvas.getContext('2d');
			ctx.clearRect(0,0,mouseCanvas.width,mouseCanvas.height);
			ctx.font = '14px serif';
			ctx.fillText(text,2,13);
			ctx.fillText(text2,2,24);
		}
		mouseCanvas.writeText('Click for','pointer lock');
      	mouseCanvas.onmousedown = function(e){
			mouseCanvas.requestPointerLock();
      	};
		mouseCanvas.plc = function(e){
			if(document.pointerLockElement === mouseCanvas || document.mozPointerLockElement === mouseCanvas){
				console.log('on');
				document.onmousemove = mouseCanvas.updateMousePosition;
				mouseCanvas.writeText('Press ESC','to exit');
			}else{
				console.log('off');
				document.onmousemove = null;
				mouseCanvas.writeText('Click for','pointer lock');
			}
		}
		document.addEventListener('pointerlockchange',mouseCanvas.plc, false);
		document.addEventListener('mozpointerlockchange',mouseCanvas.plc, false);
		  
		mouseCanvas.updateMousePosition = function(e){
			mouseCanvas.send(e.movementX,e.movementY,e.buttons);
			mouseCanvas.writeText('Press ESC',`(${e.movementX},${e.movementY})`);
			let ctx = mouseCanvas.getContext('2d');
			ctx.strokeStyle='#F00';
			ctx.lineWidth=5;
			ctx.lineCap='round';
			ctx.beginPath();
			ctx.moveTo(53,22);
			ctx.lineTo(53+e.movementX/10,22+e.movementY/10);
			ctx.stroke();
		}
		mouseCanvas.send = function(x,y,z){
			sendToRos(jsw['topic'],{
				x:x,
				y:y,
				z:z
			},'_mouse');
		}
    break;
	case  '_mic':
		var unmuteImg = new Image();
		unmuteImg.src = 'unmute.svg';
		var muteImg = new Image();
		muteImg.src = 'mute.svg';
		var ele = newWidget.querySelector('#mic_ap');
		ele.isMuted = true;
		ele.showMute = () => {
			mute();
			ele.querySelector('.imshow').style.display = 'unset';
			ele.querySelector('.imhide').style.display = 'none';
		};
		ele.showUnmute = () => {
			var jsw = widgetArray[indexMap[thisID]];	
			if(jsw) if(jsw.topic){
				unmute(jsw.topic);
				ele.querySelector('.imshow').style.display = 'none';
				ele.querySelector('.imhide').style.display = 'unset';
			}
		};
		ele.updateImage = (muted) => {
			if(muted) ele.showMute();
			else ele.showUnmute();
		};
		ele.toggle = () => {
			var jsw = widgetArray[indexMap[thisID]];	
			if(jsw) if(jsw.topic){
				ele.isMuted = !ele.isMuted;
				ele.updateImage(ele.isMuted);
				console.log('mic toggled');
			}
		};
		ele.addEventListener('mouseup',()=>{
			ele.toggle();
		});
		if(!audioStream) initMic();
	break;
	case  '_speaker':
		var unmuteImg = new Image();
		unmuteImg.src = 'unspeak.svg';
		var muteImg = new Image();
		muteImg.src = 'speak.svg';
		var ele = newWidget.querySelector('#speaker_ap');
		ele.isMuted = true;
		ele.showMute = () => {
			var jsw = widgetArray[indexMap[thisID]];	
			if(jsw) if(jsw.topic){
				socket.emit('muteRobotMic',jsw.topic);
				ele.querySelector('.imshow').style.display = 'unset';
				ele.querySelector('.imhide').style.display = 'none';
			}
		};
		ele.showUnmute = () => {
			var jsw = widgetArray[indexMap[thisID]];	
			if(jsw) if(jsw.topic){
				socket.emit('unmuteRobotMic',jsw.topic,jsw.id);
				ele.querySelector('.imshow').style.display = 'none';
				ele.querySelector('.imhide').style.display = 'unset';
			}
		};
		ele.updateImage = (muted) => {
			if(muted) ele.showMute();
			else ele.showUnmute();
		};
		ele.toggle = () => {
  			var jsw = widgetArray[indexMap[thisID]];		
			console.log('jsw',jsw,jsw.topic);
			if(jsw) if(jsw.topic){
				ele.isMuted = !ele.isMuted;
				ele.updateImage(ele.isMuted);
				soundBuffers = [];
				console.log('toggled');
				console.log(jsw.topic);
			}
		};
		ele.addEventListener('mouseup',()=>{
			console.log('ok');
			if(!playbackContext) initSpeaker();
			ele.toggle();
		});
	break;
  }
}
//returns the widget-clone as a dragable object
function widgetFromId(id){
  let itm = document.getElementById(id);
  if(!itm){
	console.log(`widget type ${id} doesn't exist in this version of UI`);
	return;
  }
  let cln = itm.cloneNode(true);
  cln.className = 'panel dragable';
  cln.style.zIndex=60;
  if(id == '_box') cln.style.zIndex = 5;
  cln.querySelector('#header').style ='padding:11px;';
  cln.querySelector('#header').childNodes[0].data = '';
  cln.id='';
  //show the gear icon to allow for configuration of that widget
  cln.querySelector('#configButton').style.display = 'inline-block';
  //add initalize canvases on widget
  let canvas = cln.querySelector('#canvas_ap');
  if(canvas){
    initJoystick(canvas);
    drawJoystick(canvas,0,0,false);
  }
  canvas = cln.querySelector('#gauge_ap');
  if(canvas){
    drawGauge(canvas,0);
  }
  canvas = cln.querySelector('#arm_ap');
  if(canvas){
    drawArm(canvas);
  }
  dragElement(cln);
  document.getElementById("body").appendChild(cln);
  return cln;
}
function updateIndexMap(){
  indexMap=[];
  for(let i = 0; i < widgetArray.length; i++){
    indexMap[widgetArray[i].id] = i;
  }
}
function updateTopicMapIndex(){
  topicMapIndex=[];
  for(let i = 0; i < widgetArray.length; i++){
    topicMapIndex[widgetArray[i].topic] = widgetArray[i].id;
  }
}
function generateUniqueId(index){
  for(let i = 0; i < 500; i++){
    if(typeof indexMap[i] === "undefined"){
      indexMap[i] == index;
      console.log('empty id found: ' + i);
      return i;
      break;
    }
  }
}

function sendToRos(topic,data,type){
	//console.log('ros out',topic,data,type);
  if(topic != undefined && topic != '/' && topic != ''){
    data.topic = topic;
    data.type = type;
	socket.emit('ROSCTS',data);//ros client to server
  }
}

//widgetArray methods
function addWidget(data){
  widgetArray.push(data);
  updateIndexMap();
  updateTopicMapIndex();
  sendWidgetsArray();
};
function moveWidget(data){
  //data is an object of the element's position
  for(let i = 0; i < widgetArray.length; i++){
    if(widgetArray[i]['id'] == data.id){
      widgetArray[i]['left'] = data.x;
      widgetArray[i]['top'] = data.y;
      break;
    }
  }
  //sendWidgetsArray();
}
function resizeWidget(data){
  //data is an object of the element's position
  for(let i = 0; i < widgetArray.length; i++){
    if(widgetArray[i]['id'] == data.id){
      widgetArray[i]['w'] = data.x;
      widgetArray[i]['h'] = data.y;
      break;
    }
  }
  sendWidgetsArray();
}
function deleteWidget(data){
  //data is the json object of the new widget
  for(let i = 0; i < widgetArray.length; i++){
    if(widgetArray[i]['id'] == data){
      widgetArray.splice(i,1);
      break;
    }
  }
  updateIndexMap();
  updateTopicMapIndex();
  sendWidgetsArray();
}
function sendWidgetsArray(){
  socket.emit('WCTS',widgetArray);//widgets client to server
}
