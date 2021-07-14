let recorder;
let audioStream;
let isMuted = true;
var micTopic;

var playbackContext;
var buf;
function initMic(){
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
}
function initSpeaker() {
	if (!window.AudioContext) {
  	  if (!window.webkitAudioContext) {
   	     alert("AudioContext is not supported on this browser");
    	 return;
   	  }
   	  window.AudioContext = window.webkitAudioContext;
  	}

    playbackContext = new AudioContext({sampleRate: 32000});
	console.log('audioContext',playbackContext);
}
function setMicTopic(){
	for(let i = 0; i < widgetArray.length; i++){
		if(widgetArray[i].type == '_mic'){
			micTopic = widgetArray[i].topic;
			break;
		}
	}
}
function unmute(){
	setMicTopic();
	const context = window.AudioContext;
	const audioContext = new context({sampleRate:32000});
	const volume = audioContext.createGain();
	const audioInput = audioContext.createMediaStreamSource(audioStream);
	audioInput.connect(volume);
	const bufferSize = 1024;
	recorder = audioContext.createScriptProcessor.call(audioContext,bufferSize,1,1);


	const leftChannel = [];
	recorder.onaudioprocess = function(event){
	const samples = event.inputBuffer.getChannelData(0);
	//const leftChannel = new Float32Array(samples);
	const leftChannel = new Array(samples.length);
	for (let i = 0; i < samples.length; i++)
	{
		 let tmp = Math.max(-1,Math.min(1,samples[i]));
		 tmp = tmp < 0 ? (tmp * 0x8000) : (tmp * 0x7FFF);
		 tmp = tmp / 256;
		
		 leftChannel[i] = tmp+256;
	}
	//sendToRos(micTopic,{value:arrayBufferToBase64(PCM16iSamples)},'_mic');
	sendToRos(micTopic,{value:leftChannel},'_mic');
	};
	// we connect the recorder
	volume.connect(recorder);
	// start recording
	recorder.connect(audioContext.destination);
	console.log('the mic is unmuted');
}
function playByteArray(byteArray){
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

//playing audio on the browser
//plays arraybuffer 16000 16 bit
function writeToAudioPlayer(soundBuffer){
	let sound = new Int8Array(soundBuffer);
    let frameCount = sound.byteLength/2;
	var myAudioBuffer = playbackContext.createBuffer(1, frameCount, 16000);
	var nowBuffering = myAudioBuffer.getChannelData(0,16,16000);
	for (var i = 0; i < frameCount; i++) {
		//var word = (sound[i * 2] & 0xff) + ((sound[i * 2 + 1] & 0xff) << 8);
		//nowBuffering[i] = ((word + 32768) % 65536 - 32768) / 32768.0;
		
		var word = (sound[i * 2] & 0xff) + ((sound[i * 2+1] & 0xff) << 8);
		nowBuffering[i] = ((word + 32768) % 65536 - 32768) / 32768.0;
	}
	var source = playbackContext.createBufferSource();
	source.buffer = myAudioBuffer;
	source.connect(playbackContext.destination);
	source.start();
}