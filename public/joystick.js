const redrawEvent = new Event('redraw');
//nel = not event listener (used for only displaying joystick)
function initJoystick(c,nel = false){
  var mouse = {x:0,y:0}, oldNormalPos = {x:0,y:0};
  var myTopic;
  var listenToGamepad;
  var myMousemove, pendingMouseup = false;
  var ctx = c.getContext("2d");
  c.width = 200;
  c.height = 130;
	
  c.addEventListener('redraw', function (e) {
   drawJoystick(c,0,0);
  }, false);
  
  if(!nel){
  c.addEventListener('mousedown', e => {
	pendingMouseup = true;
    if(c.parentNode.id != '_joystick') myTopic = widgetArray[indexMap[c.parentNode.id]]['topic'];
    mouse = getStick(e);
    drawJoystick(c,mouse.x,mouse.y);
    document.onmousemove = move;
  });
  document.addEventListener('mouseup', e => {
	if(pendingMouseup){
		pendingMouseup = false;
		drawJoystick(c,0,0);
		document.onmousemove = null;
	}
  });
  function move(e){
	mouse = getStick(e);
	if(mouse.pressed){
		drawJoystick(c,mouse.x,mouse.y);
	}
  }
}

 function getStick(e){
    if(e.buttons == 1){
      var rect = c.getBoundingClientRect();
      let m = {x:e.clientX-rect.left-c.width/2,y:e.clientY-rect.top-c.height/2,pressed:true};
      return(m);
    }
    else{
      return({x:0,y:0});
    }
  }
}
//type true if using mouse screen coords, false if using -1,1
function drawJoystick(c,x,y,type=true){
	console.log(editing);
  if(!editing){
  var parent = c.parentNode;
  var myTopic;
  if(parent.id != '_joystick' && parent.id != '') myTopic = widgetArray[indexMap[parent.id]].topic;
  else myTopic = '';
  var normalPos = {x:0,y:0};
  let ctx = c.getContext("2d");
  let mid = {x:c.width/2,y:c.height/2};
  let smallestRadius = Math.min(mid.x,mid.y);
  let outerRad = smallestRadius-10;
  //using screen x and y
  if(type){
    let hypot = Math.hypot(x,y);
    if(hypot == 0) hypot = 1;
    normalPos = {x:0,y:0};
    normalPos.x = x/hypot;
    normalPos.y = y/hypot;

    if(hypot > outerRad){
      x = normalPos.x*outerRad;
      y = normalPos.y*outerRad;
    }
  }
  else{
    let hypot = Math.hypot(x,y);
    normalPos = {x:x,y:y};
    if(hypot > 1){
      normalPos = {x:x/hypot,y:y/hypot};
      x/=hypot;
      y/=hypot;
    }
    x *= outerRad-10;
    y *= outerRad-10;
  }
  ctx.clearRect(0, 0, c.width, c.height);
  //outside ring
  ctx.beginPath();
  ctx.arc(mid.x, mid.y, outerRad, 0, 2 * Math.PI);
  ctx.fillStyle = "#828282";
  ctx.fill();
  //inside 'stick'
  ctx.beginPath();
  ctx.arc(mid.x+x, mid.y+y, smallestRadius/4, 0, 2 * Math.PI);
  ctx.fillStyle = "#4d4d4d";
  ctx.fill();
  sendToRos(myTopic,normalPos,'_joystick');
  // if(Math.abs(normalPos.x - oldNormalPos.x) > 0.02 || Math.abs(normalPos.y - oldNormalPos.y) > 0.02){
  //   normalPos.y *= -1;
  //   sendToRos(myTopic,normalPos);
  // }
  }
}
