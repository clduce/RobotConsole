let recorder;
let audioStream;
var playbackContext;

//called once to initialize the microphone
function initMic(){
	if(window.location.protocol !== "https:") return;
	audioStream = true; //make sure none of the other widgets try to init an audioStream at the same time
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

//called once to initialize the speaker
function initSpeaker() {
	if (!window.AudioContext) {
  	  if (!window.webkitAudioContext) {
   	     alert("AudioContext is not supported on this browser");
    	 return;
   	  }
   	  window.AudioContext = window.webkitAudioContext;
  	}

    playbackContext = new AudioContext({sampleRate:32000});
	console.log('audioContext',playbackContext);
	
}

//scans through every mic widget and toggles it if it's on
function muteAllMicsNotOnThisTopic(topic){
	console.log('muting all others');
	for(let i = 0; i < widgetArray.length; i++){
		let w = widgetArray[i];
		let newWidget = document.getElementById(w.id);
		if(w.topic) if(w.type == '_mic' && w.topic != topic){
			let m = newWidget.querySelector('#mic_ap');
			if(m) if(!m.isMuted) m.toggle();
		}
	}
}

//called by a mic widget
//mutes all other mic widgets
//starts immediatly sending audio messages to ros
function unmute(micTopic){
	//mute all other mics
	muteAllMicsNotOnThisTopic(micTopic);
	
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
	sendToRos(micTopic,{value:leftChannel},'_mic');
	};
	// we connect the recorder
	volume.connect(recorder);
	// start recording
	recorder.connect(audioContext.destination);
}

//shut down the mic
function mute(){
  if(recorder){
	  recorder.disconnect();
	  audioInputBuffer = [];
  }
}

//playing audio on the browser
//plays arraybuffer 16000 16 bit
//audio is stored in a circular buffer, when the buffer is full, the audio is scheduled and played back async.
//writeToAudioPlayer is called when a new packet of audio arives in main.js socket.on('telem')
var soundBuffers = [];
var lastTime= 0;
function writeToAudioPlayer(buffer){
	soundBuffers.push(buffer);
	if(soundBuffers.length > 20){
		for(let i = 0; i < soundBuffers.length; i++){
			if(soundBuffers[i] && playbackContext) playArrayBuffer(soundBuffers[i],playbackContext.currentTime+i*0.01,playbackContext.currentTime+(i+1)*0.01);
		}
		soundBuffers = [];
	}
}
function playArrayBuffer(buffer,time,stime){
	if(!playbackContext) return;
	let sound = new Int8Array(buffer);
    let frameCount = sound.byteLength/2;
	var myAudioBuffer = playbackContext.createBuffer(1, frameCount, 16000);
	var nowBuffering = myAudioBuffer.getChannelData(0,16,16000);
	for (var i = 0; i < frameCount; i++) {
		var word = (sound[i * 2] & 0xff) + ((sound[i * 2+1] & 0xff) << 8);
		nowBuffering[i] = ((word + 32768) % 65536 - 32768) / 32768.0;
		//nowBuffering[i] = (word / 32768.0 - 1);
	}
	var source = playbackContext.createBufferSource();
	source.buffer = myAudioBuffer;
	source.connect(playbackContext.destination);
	source.start(time);
	source.stop(stime);
}
