socket = io(window.location.hostname + ':' + window.location.port);
var mask = document.getElementById('mask');
let settings = {};
let camcount = 1;
let gotSettings = false, gotThumbs = false;
socket.on('settings', (data) => {
	if(!gotSettings){
		settings = data;
		gotSettings = true;
	}
	hideMessage();
});
socket.on('hardcoded_settings',function(data){
	if(!data.show_config_settings) document.body.innerHTML = '';
});

//recieve camera count
socket.on('makeThumbs',(data) => {
	if(!gotThumbs){
		camcount = data;
		document.getElementById('camcount').innerText = 'Cameras ('+camcount+' detected)';
		gotThumbs = true;
		addCams(data);
		populateConfig(settings['config']);
	}
	document.getElementById('header').innerText = 'Robot Settings';
});

//fills in the config fields with JSON object data
/*
 * {
 * 		"consoleName":"Robot Console",
 * 		"loadInEditMode":true,
 * 		"background":"#000",
 * 		"snaptogrid":false,
 * 		"wifilist":[
 * 			{
 * 				"ssid":"ssid",
 * 				"password":"password"
 * 			},
 * 			{
 * 				"ssid":"ssid",
 * 				"password":"password"
 * 			},
 * 			{
 * 				"ssid":"ssid",
 * 				"password":"password"
 * 			},
 * 			{
 * 				"ssid":"ssid",
 * 				"password":"password"
 * 			},
 * 			{
 * 				"ssid":"ssid",
 * 				"password":"password"
 * 			}
 * 		],
 * 		"hotspot":{
 * 			"ssid":"ssid",
 *  		"password":"password",
 *  		"ipaddress":"ipaddress"
 * 		},
 * 		"cams":{
 * 			"presets": [
 * 				{
 * 					"width":"320",
 * 					"height":"240",
 * 					"quality":100,
 * 					"name":"low res"
 * 				}
 * 			],
 * 			"camsettings":[
 * 				{
 * 					"preset":0,
 * 					"name":"pi cam"
 * 				}
 * 			]
 * 		}
 * }
*/
function populateConfig(data){
	for(let i = 0; i < 5; i++){
		document.getElementsByClassName('wifilist_ssid')[i].value = data['wifilist'][i]['ssid'];
		document.getElementsByClassName('wifilist_password')[i].value = data['wifilist'][i]['password'];
	}
	document.getElementsByClassName('hotspot_ssid')[0].value = data['hotspot']['ssid'];
	document.getElementsByClassName('hotspot_password')[0].value = data['hotspot']['password'];
	document.getElementsByClassName('hotspot_ip')[0].value = data['hotspot']['ipaddress'];
	if(!data['cams']) data['cams'] = [];
	for(let i = 0; i < document.getElementsByClassName('cams_width').length; i++){
		if(!data['cams'][i]) data['cams'][i] = {width:320,height:240,quality:95,fps:30};
		document.getElementsByClassName('cams_width')[i].value = data['cams'][i]['width'];
		document.getElementsByClassName('cams_height')[i].value = data['cams'][i]['height'];
		document.getElementsByClassName('cams_quality')[i].value = data['cams'][i]['quality'];
		//document.getElementsByClassName('cams_fps')[i].value = data['cams'][i]['fps'];
	}
	document.getElementsByClassName('consoleName')[0].value = data['consoleName'];
	document.getElementsByClassName('loadInEditMode')[0].checked = data['loadInEditMode'];
	document.getElementsByClassName('background')[0].value = data['background'];
	document.getElementsByClassName('snaptogrid')[0].checked = data['snaptogrid'];
	console.log('done loading settings');
}
function generateConfig(){
	let data = {};
	data['wifilist'] = [];
	for(let i = 0; i < 5; i++){
		data['wifilist'][i] = {};
		data['wifilist'][i]['ssid'] = document.getElementsByClassName('wifilist_ssid')[i].value;
		data['wifilist'][i]['password'] = document.getElementsByClassName('wifilist_password')[i].value;
	}
	data['hotspot'] = {};
	data['hotspot']['ssid'] = document.getElementsByClassName('hotspot_ssid')[0].value;
	data['hotspot']['password'] = document.getElementsByClassName('hotspot_password')[0].value;
	data['hotspot']['ipaddress'] = document.getElementsByClassName('hotspot_ip')[0].value;
	data['cams'] = [];
	for(let i = 0; i < document.getElementsByClassName('cams_width').length; i++){
		data['cams'][i] = {};
		data['cams'][i]['width'] = document.getElementsByClassName('cams_width')[i].value;
		data['cams'][i]['height'] = document.getElementsByClassName('cams_height')[i].value;
		data['cams'][i]['quality'] = document.getElementsByClassName('cams_quality')[i].value;
		//data['cams'][i]['fps'] = document.getElementsByClassName('cams_fps')[i].value;
	}
	data['consoleName'] = document.getElementsByClassName('consoleName')[0].value;
	data['loadInEditMode'] = document.getElementsByClassName('loadInEditMode')[0].checked;
	data['background'] = document.getElementsByClassName('background')[0].value
	data['snaptogrid'] = document.getElementsByClassName('snaptogrid')[0].checked;
	return data;
}
function addCams(c){
	for(let i = 0; i < c; i++){
		let html = "<p class='inputLabel'>Width (px)</p>"+
		"<input class='cams_width'></input>"+
		"<p class='inputLabel'>Height (px)</p>"+
		"<input class='cams_height'></input>"+
		"<p class='inputLabel'>JPEG Quality (0 - 100)</p>"+
		"<input class='cams_quality'></input>"+
		//"<p class='inputLabel'>FPS (0.5 - 60)</p>"+
		"<br>";
		document.getElementById('cams').insertAdjacentHTML('beforeend',html);
	}
}
function sendToServer(){
	let data = generateConfig();
	document.getElementById('header').innerText = 'Sending...';
	socket.emit('configSettings',data,(confirmation)=>{
		socket.emit('WCTS',settings['widgets'],(confirmation)=>{
			console.log('sent to server');
			document.getElementById('header').innerText = 'Success';
			return;
		});
	});
}
function exportFile(){
	settings['config'] = generateConfig();
	
	var data = encode( JSON.stringify(settings, null, 4) );
    var blob = new Blob( [ data ], {
        type: 'application/octet-stream'
    });
    url = URL.createObjectURL( blob );
    var link = document.createElement( 'a' );
    link.setAttribute( 'href', url );
    link.setAttribute( 'download', 'RobotConsole.json' );
    var event = document.createEvent( 'MouseEvents' );
    event.initMouseEvent( 'click', true, true, window, 1, 0, 0, 0, 0, false, false, false, false, 0, null);
    link.dispatchEvent( event );
}
function encode(s) {
    var out = [];
    for ( var i = 0; i < s.length; i++ ) {
        out[i] = s.charCodeAt(i);
    }
    return new Uint8Array( out );
}
function loadJsonFile(me){
	console.log('got file');
	var reader = new FileReader();
	reader.addEventListener('load',function(){
		var result = JSON.parse(reader.result);
		console.log(result);
		populateConfig(result['config']);
		settings['widgets']= result['widgets'];
	});
	reader.readAsText(me.files[0]);
}
function backToConsole(){
	if(gotSettings) sendToServer();
	window.location.href = window.location.href.replace('/config.html','');
}
function showMessage(text){
	mask.style.display = 'inline';
	document.getElementById('messagePanel').style.display = 'flex';
	document.getElementById('messagePanelText').innerText = text;
}
function hideMessage(){
	mask.style.display = 'none';
	document.getElementById('messagePanel').style.display = 'none';
}
