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
	document.ontouchmove = touchmove;
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
  function touchmove(e){
	  console.log(e);
	//mouse = getStick(e);
	//if(mouse.pressed){
	//	drawJoystick(c,mouse.x,mouse.y);
	//}
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
    normalPos = {x:x/hypot,y:y/hypot};

    if(hypot > outerRad){
      x = normalPos.x*outerRad;
      y = normalPos.y*outerRad;
    }
    normalPos = {x:x/outerRad,y:y/outerRad};
    
    //console.log('using screen',normalPos);
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
  normalPos.y *= -1;
  sendToRos(myTopic,normalPos,'_joystick');
  // if(Math.abs(normalPos.x - oldNormalPos.x) > 0.02 || Math.abs(normalPos.y - oldNormalPos.y) > 0.02){
  //   normalPos.y *= -1;
  //   sendToRos(myTopic,normalPos);
  // }
  }
}

//gauge widget
//let opts= {min:-50,max:50,bigtick:10,smalltick:5, title:'CPU temp'};
//drawGuage(document.getElementById("guageWidget"),0,opts);
function drawGauge(c,v,format){
	if(!format) format = {};
	opts = JSON.parse(c.getAttribute("data-config"));
	opts.min=Number(opts.min);
	opts.max=Number(opts.max);
	opts.bigtick=Number(opts.bigtick);
	opts.smalltick=Number(opts.smalltick);
	//v=v|opts.min;
	if(v == undefined) v = opts.min;
    y=c.height/2;
    x=c.width/2;
    r=Math.min(x,y)-10;
    ctx = c.getContext("2d");
	
	ctx.clearRect(0,0,2*x,2*y);
	ctx.beginPath();
    ctx.arc(x, y, r, 0, 2 * Math.PI);
    ctx.fillStyle = '#FFF';
    ctx.fill();
    
    //outline
    ctx.strokeStyle = '#000';
    ctx.beginPath();
    ctx.arc(x, y, r, 0, 2 * Math.PI);
    ctx.closePath();
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(x, y, r-r*0.05, 0, 2 * Math.PI);
    ctx.closePath();
    ctx.lineWidth = r*0.1;
    ctx.strokeStyle = '#b8b8b8';
    ctx.stroke();

    let h = -Math.PI*1.4;
    let ofst = + Math.PI*0.8
    let vr = opts.max-opts.min;//value range
    let nt = (vr/opts.bigtick) * (opts.smalltick)  //number of ticks
    let sth = h/nt; //small tick height
    let std = vr/nt;//small tick delta
    let zpos = y-(-opts.min/std)*sth;//pixel y where the scale is zero
    let cbh = zpos-(y-(v/std-opts.min/std)*sth);//calculated bar height
    let or = r*0.15;
    let ir = r*0.3;

    ctx.lineCap = 'round';
    ctx.font = (r*0.05+10)+"px Arial";
    ctx.fillStyle = "#5c5c5c";
    ctx.textAlign = 'center';

    for(let i = 0; i <= nt; i++){
      ctx.beginPath();
      if(i%(opts.smalltick) == 0){
        ctx.moveTo(x+Math.cos(-i*sth+ofst)*(r-or),y+Math.sin(-i*sth+ofst)*(r-or));
        ctx.lineTo(x+Math.cos(-i*sth+ofst)*(r-ir),y+Math.sin(-i*sth+ofst)*(r-ir));
        ctx.lineWidth = 2;
        ctx.strokeStyle = "#5c5c5c";
        ctx.fillText(Math.round( (i*std+opts.min) * 100 + Number.EPSILON ) / 100,x+Math.cos(-i*sth+ofst)*(r*0.9-ir),y+Math.sin(-i*sth+ofst)*(r*0.9-ir)+5);
      }
      else{
        ctx.moveTo(x+Math.cos(-i*sth+ofst)*(r-or),y+Math.sin(-i*sth+ofst)*(r-or));
        ctx.lineTo(x+Math.cos(-i*sth+ofst)*(r-ir*0.8),y+Math.sin(-i*sth+ofst)*(r-ir*0.8));
        ctx.lineWidth = 1.5;
        ctx.strokeStyle = "#b8b8b8";
      }
      ctx.stroke();
    }
    ctx.fillStyle= '#000';
    ctx.font = (r*0.3)+"px Arial";'px Arial';
    ctx.fillText(formatNumber(v,format),x,y+r*0.7);
    ctx.fillStyle= '#666';
    ctx.font = (r*0.15)+"px Arial";'px Arial';
    ctx.fillText(opts.title,x,y-r*0.3);

    let a = ((v-opts.min)/(opts.max-opts.min)) * -h + ofst;
    let tw = r*0.04;
    ctx.beginPath();
    ctx.moveTo(x+Math.cos(a-3.14/2)*tw,y+Math.sin(a-3.14/2)*tw);
    ctx.lineTo(x+Math.cos(a)*r*0.8,y+Math.sin(a)*r*0.8);
    ctx.lineTo(x+Math.cos(a+3.14/2)*tw,y+Math.sin(a+3.14/2)*tw);
    ctx.fillStyle = 'rgba(255, 0, 0,0.4)';
    ctx.strokeStyle = 'rgb(255, 0, 0,0.5)';
    ctx.fill();
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(x, y, r*0.15, 0, 2 * Math.PI);
    ctx.lineWidth = 2;
    ctx.fillStyle = '#548aff';
    ctx.fill();
	
    ctx.strokeStyle = '#AAA';
    ctx.stroke();
  }

function drawArm(canvas,arms,angleArray){
	if(arms == undefined) arms = [{mode:1,data:60,armlength:5,color:'#000000'},{mode:1,data:-90,armlength:3,color:'#00FF00'}];
	c = canvas.getContext("2d");
	c.clearRect(0,0,canvas.width,canvas.height);
	let rotation = 0;
	let endpos = {x:0,y:0};
	let offset = {x:20,y:canvas.height/2 + 20};
	for(let i = 0; i < arms.length; i++){
		let data = 0, armlength = 3; 
		if(arms[i].mode == 0){ //if using an array index instead of fixed angle
			if(angleArray != undefined) if(angleArray[arms[i].data] != undefined) data = angleArray[arms[i].data] * -0.01745;
		}
		else data = arms[i].data * -0.01745; //convert degrees to radians
		armlength = arms[i].armlength * 20; //arm length multiplier
		
		c.lineWidth = 18;
		c.lineCap = 'round';
		c.strokeStyle = arms[i].color;
		c.beginPath();
		c.moveTo(offset.x+endpos.x, offset.y+endpos.y);
		c.lineTo(offset.x+endpos.x+Math.cos(data+rotation)*armlength, offset.y+endpos.y+Math.sin(data+rotation)*armlength);
		c.stroke();

		c.lineWidth = 7;
		c.lineCap = 'round';
		c.beginPath();
		c.arc(offset.x+endpos.x, offset.y+endpos.y, 12, 0, 2 * Math.PI);
		c.fill();

		endpos.x += Math.cos(data+rotation)*armlength;
		endpos.y += Math.sin(data+rotation)*armlength;
		rotation += data;
	}
}
