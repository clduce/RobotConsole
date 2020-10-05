var editing = false;//this is true when you drag elements
var mask = document.getElementById('mask');
var configWindow = document.getElementById('configWindow');
var configIsOpen = false,elementOpenInConfig;
var driveMode = false, widgetHolderOpen = true;
var currentID, currentIndex;
var widgetArray = JSON.parse('{}');
var socket;
var loadedElements = false;//flag to check if elements have been constructed from json sent from server
var readGamepadInterval,currentGamepad,oldGamepad,lastChangedAxis;
var thisScreen = 1;
var keys = {};
var oldKeys = {};

//use same IP to connect to socket server as to connect to express
socket = io(window.location.href);

//get all config from server
socket.on('settings',function(data){
  if(!loadedElements){
  console.log(data);
  widgetArray = data['widgets'];
  updateIndexMap();
  for (let a of widgetArray){
   //generate HTML element for each widget
   if(a.screen == thisScreen) widgetFromJson(a);
  }
  loadedElements = true;
}else{
  console.log('already loaded elements');
}
});

//on video feed recieve
var videoElements = document.getElementsByClassName('image');
socket.on('image',function(data){
  for(let i = 0; i < videoElements.length; i++){
    //videoElements[i].src = data;
    videoElements[i].src = `data:image/jpeg;base64,${data}`;
  }
});
// updateTopicMapIndex();
socket.on('telem',function(data){
  console.log(widgetArray[data.id].id);
  const ele = document.getElementById(widgetArray[data.id].id).querySelector('#text_ap');
  let prefix = widgetArray[data.id].prefix;
  let postfix = widgetArray[data.id].postfix;
  ele.innerText = prefix + data.msg.data + postfix;
});

//initalize all graphical Widgets in source bar
var canvas = document.getElementById('_joystick').querySelector('#canvas_ap');
initJoystick(canvas, true);
drawJoystick(canvas,0,0);

//initalize javascript for all source elements:
var tempList = document.getElementsByClassName('source');
for (let a of tempList) {
  sourceElement(a);
}

//initalize javascript for all dragable elements:
initWidgetElements();

function initWidgetElements(){
  let tempList = document.getElementsByClassName('dragable');
  for (let a of tempList) {
    dragElement(a);
  }
}


//WIDGET BOX INTERACTION
function dragElement(elmnt) {
  var pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
  var pos1S = 0, pos2S = 0, pos3S = 0, pos4S = 0;
  //use header as draggable hold if it exists, otherwise just use the whole div
  if (elmnt.querySelector("#header")) elmnt.querySelector("#header").onmousedown = dragMouseDown;
  if (elmnt.querySelector("#resize")) elmnt.querySelector("#resize").onmousedown = scaleMouseDown;
  // elmnt.onmousedown = dragMouseDown;
  function dragMouseDown(e) {
    e = e || window.event;
    e.preventDefault();
    pos3 = e.clientX;
    pos4 = e.clientY;
    editing = true;
    document.onmouseup = closeDragElement;
    document.onmousemove = elementDrag;
  }
  function scaleMouseDown(e) {
    e = e || window.event;
    e.preventDefault();
    pos3S = e.clientX-parseInt(elmnt.style.width);
    pos4S = e.clientY-parseInt(elmnt.style.height);
    document.onmouseup = closeScaleElement;
    document.onmousemove = elementScale;
  }
  function elementDrag(e) {
    e = e || window.event;
    e.preventDefault();
    pos1 = pos3 - e.clientX;
    pos2 = pos4 - e.clientY;
    pos3 = e.clientX;
    pos4 = e.clientY;
    if(elmnt.offsetTop - pos2 < 50) elmnt.style.top = '50px';
    else elmnt.style.top = (elmnt.offsetTop - pos2) + "px";
    elmnt.style.left = (elmnt.offsetLeft - pos1) + "px";
  }
  function elementScale(e) {
    e = e || window.event;
    e.preventDefault();
    pos1S = pos3S - e.clientX;
    pos2S = pos4S - e.clientY;
    if(pos1S > -100) pos1S = -100;
    if(pos2S > -60) pos2S = -60;
    elmnt.style.height = -pos2S + "px";
    elmnt.style.width = -pos1S + "px";
    if(elmnt.querySelector('#canvas_ap')){
      var canvas = elmnt.querySelector('#canvas_ap');
      canvas.height = -pos2S-20;
      canvas.width = -pos1S;
      canvas.dispatchEvent(redrawEvent);
    }
  }
  function closeDragElement() {
    document.onmouseup = null;
    document.onmousemove = null;
    editing = false;
    if(parseInt(elmnt.style.left,10) < 245 && widgetHolderOpen){
      removeWidgetFromScreen(elmnt);
    }
    else{
      moveWidget({id:elmnt.id,x:elmnt.style.left,y:elmnt.style.top});
    }
  }
  function closeScaleElement() {
    document.onmouseup = null;
    document.onmousemove = null;
    resizeWidget({id:elmnt.id,x:elmnt.style.width,y:elmnt.style.height});
    // TODO: send to server
  }
}
function removeWidgetFromScreen(elmnt){
  elmnt.remove();
  deleteWidget(elmnt.id);
}
//turn element into source:
//the _type convention is used to determime
//what dragable item to create
function sourceElement(elmnt) {
  var newElement;
  var pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
  //use header as draggable hold if it exists, otherwise just use the whole div
  //if (elmnt.querySelector("#header")) elmnt.querySelector("#header").onmousedown = dragMouseDown;
  //else elmnt.onmousedown = dragMouseDown;
  elmnt.onmousedown = dragMouseDown;
  function dragMouseDown(e) {
    newElement = widgetFromId(elmnt.id);
    e = e || window.event;
    e.preventDefault();
    // get the mouse cursor position at startup:
    pos3 = e.clientX;
    pos4 = e.clientY;
    var scroll = document.getElementById('widgetHolder').scrollTop;
    newElement.style.top = (elmnt.offsetTop-scroll-pos2+50) + "px";
    newElement.style.left = (elmnt.offsetLeft - pos1) + "px";
    editing = true;
    document.onmouseup = closeDragElement;
    document.onmousemove = elementDrag;
  }
  function elementDrag(e) {
    e = e || window.event;
    e.preventDefault();
    // calculate the new cursor position:
    pos1 = pos3 - e.clientX;
    pos2 = pos4 - e.clientY;
    pos3 = e.clientX;
    pos4 = e.clientY;
    // set the element's new position:
    if(newElement.offsetTop - pos2 < 50) newElement.style.top = '50px';
    else newElement.style.top = (newElement.offsetTop - pos2) + "px";
    newElement.style.left = (newElement.offsetLeft - pos1) + "px";
  }
  function closeDragElement(e) {
    //delete element if it's over the widget bar
    document.onmouseup = null;
    document.onmousemove = null;
    if(parseInt(newElement.style.left,10) < 245 && widgetHolderOpen){
		newElement.remove();
		editing = false;
	}
    else{
      //generate json, and give widget id
      addWidget(makeUnique(elmnt.id,newElement));
      editing = false;
    }
  }
}



//WIDGET CONFIGURATION PANEL
//open configuration settings panel for each widget
function openConfig(e){
  //load field values with JSON settings
  elementOpenInConfig = e.parentElement.parentElement;
  currentID = e.parentElement.parentElement.id;
  currentIndex = indexMap[currentID];
  lastChangedAxis = -1;
  lastChangedButton = -1;
  var type = widgetArray[currentIndex].type;
  //pull generic data from widget array into the settings
  document.getElementById('topicTitle').value = widgetArray[currentIndex]['topic'];
  //delete all the auto generated elements
  var paras = document.getElementsByClassName('specific')
  while(paras[0]) paras[0].parentNode.removeChild(paras[0]);
  switch(type){
    case '_button':
      createconfigInput('Button Label', '_button-labelText', widgetArray[currentIndex]['label']);
      createconfiglinkGamepadButton(widgetArray[currentIndex]);
      createconfiglinkKeys(widgetArray[currentIndex]);
    break;
    case '_joystick':
      createconfiglinkGamepadAxis(widgetArray[currentIndex]);
      createconfiglinkKeys(widgetArray[currentIndex],['up','left','down','right']);
    break;
    case '_checkbox':
      createconfigInput('Label', 'Label', widgetArray[currentIndex]['label']);
      createCheckbox('Initial State', 'initialState', widgetArray[currentIndex]['initial']);
      createconfiglinkGamepadButton(widgetArray[currentIndex]);
      createconfiglinkKeys(widgetArray[currentIndex],['hotkey']);
    break;
    case '_value':
      createconfigDataWrapper(widgetArray[currentIndex]);
      createSelect('Subscribe to message type', 'msgType', widgetArray[currentIndex]['msgType'] ,['std_msgs/String','std_msgs/Float32','std_msgs/Float64','std_msgs/Int16','std_msgs/Int32','std_msgs/Int64','std_msgs/Bool']);
    break;
  }
  mask.style.display='inline';
  configWindow.style.display='inline';
  configIsOpen = true;
}
function applyConfigChanges(){
  //the widget were applying settings on
  var localWidget = document.getElementById(currentID);
  var topic = cleanTopicName(document.getElementById('topicTitle').value);
  widgetArray[currentIndex].topic = topic;
  var type = widgetArray[currentIndex].type;

  switch(type){
    case '_button':
      widgetArray[currentIndex]['label'] = document.getElementById('_button-labelText').value;
      localWidget.querySelector('#button_ap').innerText = widgetArray[currentIndex]['label'];
      widgetArray[currentIndex]['useGamepad'] = document.getElementById('useGamepad').checked;
      widgetArray[currentIndex]['useKeys'] = document.getElementById('useKeys').checked;
      widgetArray[currentIndex]['usekey_hotkey'] = document.getElementById('usekey_hotkey').value;
      if(lastChangedButton != -1) widgetArray[currentIndex]['useButton'] = lastChangedButton;
    break;
    case '_checkbox':
      widgetArray[currentIndex]['label'] = document.getElementById('Label').value;
      widgetArray[currentIndex]['initial'] = document.getElementById('initialState').checked;
      localWidget.querySelector('#checkbox_text_ap').innerText = widgetArray[currentIndex]['label'];
      widgetArray[currentIndex]['useGamepad'] = document.getElementById('useGamepad').checked;
      widgetArray[currentIndex]['useKeys'] = document.getElementById('useKeys').checked;
      widgetArray[currentIndex]['usekey_hotkey'] = document.getElementById('usekey_hotkey').value;
      if(lastChangedButton != -1) widgetArray[currentIndex]['useButton'] = lastChangedButton;
    break;
    case '_joystick':
      widgetArray[currentIndex]['useGamepad'] = document.getElementById('useGamepad').checked;
      widgetArray[currentIndex]['useKeys'] = document.getElementById('useKeys').checked;
      widgetArray[currentIndex]['usekey_up'] = document.getElementById('usekey_up').value;
      widgetArray[currentIndex]['usekey_left'] = document.getElementById('usekey_left').value;
      widgetArray[currentIndex]['usekey_down'] = document.getElementById('usekey_down').value;
      widgetArray[currentIndex]['usekey_right'] = document.getElementById('usekey_right').value;
      if(lastChangedAxis != -1) widgetArray[currentIndex]['useAxis'] = lastChangedAxis;
    break;
    case '_value':
      widgetArray[currentIndex]['prefix'] = document.getElementById('textInput1').value;
      widgetArray[currentIndex]['postfix'] = document.getElementById('textInput2').value;
      widgetArray[currentIndex]['msgType'] = document.getElementById('msgType').value;
    break;
  }
  mask.style.display='none';
  configWindow.style.display='none';
  configIsOpen = false;
  updateTopicMapIndex();
  sendWidgetsArray();
}
function cleanTopicName(str){
	return str;
}
//dynamically creates custom config settings. input is the content id ex _button.labelText
function createconfigInput(labelText, inputID, inputvalue){
  var label = document.createElement("h1");
  label.className = 'settingsLabel specific';
  label.style.margin.top = '20px';
  label.innerText = labelText;
  var content = document.createElement("input");
  content.className = 'settingInput specific';
  content.id=inputID;
  content.value = (inputvalue==undefined?'':inputvalue);
  configWindow.appendChild(label);
  configWindow.appendChild(content);
}
function createCheckbox(labelText, inputID, inputvalue){
  var label = document.createElement("h1");
  label.className = 'settingsLabel specific';
  label.style.margin.top = '20px';
  label.innerHTML = labelText + " <input id='"+inputID+"'class='specific' type='checkbox'></input>";
  var content = document.createElement("input");
  configWindow.appendChild(label);
  document.getElementById(inputID).checked = inputvalue;
}
//inputvalue is currently selected value from opts
function createSelect(labelText, inputID, inputvalue, opts){
  var label = document.createElement("h1");
  label.className = 'settingsLabel specific';
  label.style.margin.top = '20px';
  label.innerHTML = labelText;
  configWindow.appendChild(label);
  var content = document.createElement("select");
  content.className = 'specific';
  content.id=inputID;
  for(let i = 0; i < opts.length; i++){
	content.innerHTML += "<option value='"+opts[i]+"'>"+opts[i]+"</option>";
  }
  configWindow.appendChild(content);
  if(inputvalue != undefined && inputvalue != '') document.getElementById(inputID).value = inputvalue;
}
function createconfiglinkGamepadAxis(array){
  var label = document.createElement("h1");
  label.className = 'settingsLabel specific';
  label.style.margin = '20px 0px 0px 0px';
  label.innerHTML = 'Use Gamepad Input <input id="useGamepad" type="checkbox"></input>';
  label.style.margin.bottom = '0px';
  var axisText = document.createElement("p");
  axisText.className = 'specific';
  axisText.id='replaceWithCAxis';
  axisText.style.margin = '0px';
  if(array["useAxis"] == -1) axisText.innerHTML = 'wiggle a joystick on the gamepad to link...';
  else axisText.innerHTML = 'Paired to axis: '+array['useAxis'];
  configWindow.appendChild(label);
  configWindow.appendChild(axisText);
  document.getElementById('useGamepad').checked = array["useGamepad"];
}
function createconfiglinkGamepadButton(array){
  var label = document.createElement("h1");
  label.className = 'settingsLabel specific';
  label.style.margin = '20px 0px 0px 0px';
  label.innerHTML = 'Use Gamepad Input <input id="useGamepad" type="checkbox"></input>';
  label.style.margin.bottom = '0px';
  var buttonText = document.createElement("p");
  buttonText.className = 'specific';
  buttonText.id='replaceWithCButton';
  buttonText.style.margin = '0px';
  if(array["useButton"] == -1) buttonText.innerHTML = 'Press a button on the gamepad to link...';
  else buttonText.innerHTML = 'Paired to button: '+array['useButton'];
  configWindow.appendChild(label);
  configWindow.appendChild(buttonText);
  document.getElementById('useGamepad').checked = array["useGamepad"];
}
function createconfiglinkKeys(array,keylabels=['hotkey']){
  var label = document.createElement("h1");
  label.className = 'settingsLabel specific';
  label.style.margin = '20px 0px 0px 0px';
  label.innerHTML = 'Use Keyboard Input <input id="useKeys" type="checkbox"></input>';
  label.style.margin.bottom = '0px';
  configWindow.appendChild(label);
  var label2 = document.createElement("p");
  label2.className = 'specific';
  label2.style.margin = '0px';
  let innerhtml='';
  for(let i = 0; i < keylabels.length; i++){
    let value = array['usekey_'+keylabels[i]];
    if(value == undefined) value = '';
    innerhtml += keylabels[i]+' <input style="width:50px;margin-right:5px;" id="usekey_'+keylabels[i]+'" value="'+value+'"></input>';
  }
  label2.innerHTML = innerhtml;
  label2.style.margin.bottom = '0px';
  configWindow.appendChild(label2);
  document.getElementById('useKeys').checked = array["useKeys"];
}
function createconfigDataWrapper(array){
  let prefix = array.prefix;
  let postfix = array.postfix;
  if(prefix == undefined) prefix = '';
  if(postfix == undefined) postfix = '';
  let p = document.createElement('p');
  p.className = 'specific';
  p.innerHTML = "Prefix: <input id='textInput1'value='"+prefix+"'></input> Postfix: <input id='textInput2'value='"+postfix+"'></input>";
  configWindow.appendChild(p);
}

//KEYBOARD INTERFACING
document.addEventListener('keydown', (e) => {
  oldKeys = {...keys};
  keys[e.key] = true;
  getKeyboardUpdates();
});
document.addEventListener('keyup', (e) => {
  oldKeys = {...keys};
  keys[e.key] = false;
  getKeyboardUpdates();
});
function keysChanged(k){
  for(let i = 0; i < k.length; i++){
    if(oldKeys[k[i]] != keys[k[i]]) return true;
  }
  return false;
}
function getKeyboardUpdates(){
  for(let w = 0; w < widgetArray.length; w++){
    if(widgetArray[w].type == '_joystick' && widgetArray[w].useKeys){
      let ck = [widgetArray[w].usekey_up,widgetArray[w].usekey_left,widgetArray[w].usekey_down,widgetArray[w].usekey_right];
      if(keysChanged(ck)){
        let dat = {x:0,y:0};
        if(keys[ck[0]]) dat.y = -1;
        if(keys[ck[1]]) dat.x = -1;
        if(keys[ck[2]]) dat.y = 1;
        if(keys[ck[3]]) dat.x = 1;
        drawJoystick(document.getElementById(widgetArray[w].id).querySelector('#canvas_ap'),dat.x,dat.y,false);
      }
    }
    if(widgetArray[w].type == '_button' && widgetArray[w].useKeys){
      let ck = [widgetArray[w].usekey_hotkey];
      if(keysChanged(ck)){
        let dat = {x:0,y:0};
        let ele = document.getElementById(widgetArray[w].id).querySelector('#button_ap');
        if(keys[ck[0]]){
          triggerMouseEvent(ele,'mousedown');
          ele.className += " button_apPressed";
        }
        else{
          triggerMouseEvent(ele,'mouseup');
          ele.className = ele.className.replace(" button_apPressed", "");
        }
      }
    }
    if(widgetArray[w].type == '_checkbox' && widgetArray[w].useKeys){
      let ck = [widgetArray[w].usekey_hotkey];
      if(keysChanged(ck)){
        let dat = {x:0,y:0};
        let ele = document.getElementById(widgetArray[w].id).querySelector('#checkbox_ap');
        if(keys[ck[0]]){
          ele.checked = !ele.checked;
          sendToRos(widgetArray[w].topic,{pressed:ele.checked},widgetArray[w].type);
        }
      }
    }
  }
}



//GAMEPAD INTERFACING
window.addEventListener("gamepadconnected", function(e) {
  console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.",
    e.gamepad.index, e.gamepad.id,
    e.gamepad.buttons.length, e.gamepad.axes.length);
    currentGamepad = navigator.getGamepads()[0];
    oldGamepad = currentGamepad;
    readGamepadInterval = setInterval(readGamepadLoop,30);
});

window.addEventListener("gamepaddisconnected", function(e) {
  console.log("Gamepad disconnected from index %d: %s",
    e.gamepad.index, e.gamepad.id);
    clearInterval(readGamepadInterval);
});

function readGamepadLoop(){
  currentGamepad = navigator.getGamepads()[0];
  for(let i = 0; i < currentGamepad.axes.length; i++){
    if(Math.abs(currentGamepad.axes[i] - oldGamepad.axes[i]) > 0.002){
      if(configIsOpen){
        try{
          document.getElementById('replaceWithCAxis').innerText = `Paired to axis: ${lastChangedAxis}`;
        }
        catch{}
        lastChangedAxis = Math.floor(i/2);
      }
      for(let w = 0; w < widgetArray.length; w++){
        if(widgetArray[w].screen == thisScreen && widgetArray[w].type == '_joystick' && widgetArray[w].useGamepad && 'useAxis' in widgetArray[w] && widgetArray[w].useAxis == Math.floor(i/2))
        drawJoystick(document.getElementById(widgetArray[w].id).querySelector('#canvas_ap'),currentGamepad.axes[widgetArray[w].useAxis*2],currentGamepad.axes[widgetArray[w].useAxis*2+1],false);
      }
    }
  }
  for(let i = 0; i < currentGamepad.buttons.length; i++){
    if(currentGamepad.buttons[i].pressed != oldGamepad.buttons[i].pressed){
      if(configIsOpen){
        lastChangedButton = i;
        document.getElementById('replaceWithCButton').innerText = `Paired to button: ${lastChangedButton}`;
      }
      for(let w = 0; w < widgetArray.length; w++){
        if(widgetArray[w].screen == thisScreen && widgetArray[w].type == '_button' && widgetArray[w].useGamepad && 'useButton' in widgetArray[w] && widgetArray[w]['useButton'] == i){
          var ele = document.getElementById(widgetArray[w].id).querySelector('#button_ap');
          if(currentGamepad.buttons[i].pressed){
            triggerMouseEvent(ele,'mousedown');
            ele.className += " button_apPressed";
          }
          else{
            triggerMouseEvent(ele,'mouseup');
            ele.className = ele.className.replace(" button_apPressed", "");
          }
        }
        if(widgetArray[w].screen == thisScreen && widgetArray[w].type == '_checkbox' && widgetArray[w].useGamepad && 'useButton' in widgetArray[w] && widgetArray[w]['useButton'] == i){
          var ele = document.getElementById(widgetArray[w].id).querySelector('#checkbox_ap');
          if(currentGamepad.buttons[i].pressed){
            ele.checked = !ele.checked;
            sendToRos(widgetArray[w].topic,{pressed:ele.checked},widgetArray[w].type);
          }
        }
      }
    }
  }
  oldGamepad = currentGamepad;
}
function triggerMouseEvent(node, eventType) {
    var clickEvent = document.createEvent ('MouseEvents');
    clickEvent.initEvent (eventType, true, true);
    node.dispatchEvent (clickEvent);
}

//DRIVE MODE
function toggleDriveMode(){
  if(driveMode){
    driveMode = false;
    let dm = document.getElementById('driveMode');
    dm.innerText = 'Drive';

    for(let i = 0; i < widgetArray.length; i++){
      if(widgetArray[i].screen == thisScreen){
        let ele = document.getElementById(widgetArray[i].id);
        ele.style.visibility='visible';
      }
    }
    document.getElementsByClassName('toggleWidgetHolder')[0].style.visibility='visible';
  }else{
    driveMode = true;
    let dm = document.getElementById('driveMode');
    dm.innerText = 'Edit';
    hideWidgetHolder();

    for(let i = 0; i < widgetArray.length; i++){
      if(widgetArray[i].screen == thisScreen){
        let ele = document.getElementById(widgetArray[i].id);
        ele.style.visibility='hidden';
      }
    }
    document.getElementsByClassName('toggleWidgetHolder')[0].style.visibility='hidden';
  }
}
function hideWidgetHolder(){
  let me = document.getElementsByClassName('toggleWidgetHolder')[0];
  document.getElementById('widgetHolder').style.left = '-260px';
  widgetHolderOpen = false;
  me.style.left = '5px';
  me.innerText = 'Show';
}
function showWidgetHolder(){
  let me = document.getElementsByClassName('toggleWidgetHolder')[0];
  document.getElementById('widgetHolder').style.left = '0px';
  widgetHolderOpen = true;
  me.style.left = '192px';
  me.innerText = 'Hide';
}
function toggleWidgetHolder(){
  if(!driveMode){
    if(widgetHolderOpen){
      hideWidgetHolder();
    }
    else{
      showWidgetHolder();
    }
  }
}
function changeScreen(me){
  thisScreen = me.value;
  for(let i = 0; i < widgetArray.length; i++){
    if(widgetArray[i].screen == thisScreen){
      widgetFromJson(widgetArray[i]);
    }
    else{
      body.removeChild(document.getElementById(widgetArray[i].id));
    }
  }
  if(thisScreen == 1) {
	try{
	  camStream = document.getElementsByClassName('imageHidden')[0];
	  camStream.style.visibility = 'visible';
	  camStream.className = 'image';
	}
	catch(e){
	  console.log(e);
	}
  }
  else{
	  camStream = document.getElementsByClassName('image')[0];
	  camStream.style.visibility = 'hidden';
	  camStream.className = 'imageHidden';
  }
  toggleDriveMode();
  toggleDriveMode();
}
function camSelect(me){
  socket.emit('setCam',me.value);
}
