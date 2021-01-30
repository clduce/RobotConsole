const toggleElement = document.getElementById('toggleMic');
const roboToggleElement = document.getElementById('toggleRoboMic');
const UNMUTE_IMAGE = 'unmute.svg', MUTE_IMAGE = 'mute.svg';

let recorder;
let audioStream;
let isMuted = true, roboIsMuted = true;
if (navigator.getUserMedia)
{
   navigator.getUserMedia({audio: true}, function(stream){
       audioStream = stream;
       console.log('audio stream', audioStream);
   }, function(error){
       alert('Error capturing audio.');
   });
}
else
{
   alert('getUserMedia not supported in this browser.');
 }

function unmute(){
 const context = window.AudioContext;
 const audioContext = new context();
//const sampleRate = audioContext.sampleRate;
const sampleRate = 22050;
const volume = audioContext.createGain();
const audioInput = audioContext.createMediaStreamSource(audioStream);
audioInput.connect(volume);
const bufferSize = 512;
recorder = audioContext.createScriptProcessor.call(audioContext,bufferSize,1,1);


const leftChannel = [];
recorder.onaudioprocess = function(event){
  const samples = event.inputBuffer.getChannelData(0);
  const leftChannel = new Float32Array(samples);
	const PCM16iSamples = [];
	for (let i = 0; i < leftChannel.length; i++)
  	{
    	 let tmp = Math.max(-1,Math.min(1,leftChannel[i]));
    	 tmp = tmp < 0 ? (tmp * 0x8000) : (tmp * 0x7FFF);
   	  	 tmp = tmp / 256;
    	 PCM16iSamples.push(tmp-256);
  	}
  	socket.emit('audioPacket',arrayBufferToBase64(PCM16iSamples));
  };
  // we connect the recorder
  volume.connect(recorder);
  // start recording
  recorder.connect(audioContext.destination);
  console.log('the mic is unmuted');
}
function arrayBufferToBase64( buffer ) {
	var binary = '';
	var bytes = new Uint8Array( buffer );
	var len = bytes.byteLength;
	for (var i = 0; i < len; i++) {
		binary += String.fromCharCode( bytes[ i ] );
	}
	return window.btoa( binary );
}
function base64ToArrayBuffer(base64) {
    var binary_string =  window.atob(base64);
    var len = binary_string.length;
    var bytes = new Uint8Array( len );
    for (var i = 0; i < len; i++)        {
        bytes[i] = binary_string.charCodeAt(i);
    }
    return bytes.buffer;
}
function mute(){
  if(recorder){
	  recorder.disconnect();
	  audioInputBuffer = [];
	  console.log('the mic is muted');
  }
}

function toggleMic(){
  isMuted = !isMuted;
  if(isMuted){
    toggleElement.src=UNMUTE_IMAGE;
    mute();
  }
  else{
    toggleElement.src=MUTE_IMAGE;
    unmute();
  }
}

function toggleRoboMic(){
  if(roboIsMuted) unmuteRobo();
  else muteRobo();
}
function updateRoboImageStatus(){
  if(roboIsMuted){
    roboToggleElement.src='robo'+UNMUTE_IMAGE;
  }
  else{
    roboToggleElement.src='robo'+MUTE_IMAGE;
  }
}
function muteRobo(){
	socket.emit('muteRobot');
}
function unmuteRobo(){
	socket.emit('unmuteRobot');
}
socket.on('robotMuted',data=>{
	roboIsMuted = true;
	updateRoboImageStatus();
});
socket.on('robotUnmuted',data=>{
	roboIsMuted = false;
	updateRoboImageStatus();
});
socket.on('muted',data=>{
	roboIsMuted = data;
	updateRoboImageStatus();
});