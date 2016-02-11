socket = io(window.location.hostname);
let settings = {};

socket.on('settings', (data) => {
	settings = data;
	populateConfig(data['config']);
});


//fills in the config fields with JSON object data
/*
 * {
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
	return data;
}
function sendToServer(){
	let data = generateConfig();
	socket.emit('configSettings',data);
	socket.emit('WCTS',settings['widgets']);
	console.log('sent to server');
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
