const toggleElement = document.getElementById('toggleMic');
const UNMUTE_IMAGE = 'unmute.svg', MUTE_IMAGE = 'mute.svg';

let recorder;
let audioStream;
let isMuted = true;
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
 // creates the an instance of audioContext
 const context = window.AudioContext;
 const audioContext = new context();

 // retrieve the current sample rate of microphone the browser is using
const sampleRate = audioContext.sampleRate;
	console.log(sampleRate);

// creates a gain node
const volume = audioContext.createGain();

// creates an audio node from the microphone incoming stream
const audioInput = audioContext.createMediaStreamSource(audioStream);
audioInput.connect(volume);

/* From the spec: This value controls how frequently the audioprocess event is
dispatched and how many sample-frames need to be processed each call.
Lower values for buffer size will result in a lower (better) latency.
Higher values will be necessary to avoid audio breakup and glitches */
const bufferSize = 512;
recorder = audioContext.createScriptProcessor.call(audioContext,bufferSize,1,1);


const leftChannel = [];
recorder.onaudioprocess = function(event){
  const samples = event.inputBuffer.getChannelData(0);
  const leftChannel = new Float32Array(samples);
  
  const PCM16iSamples = [];
  //for (let i = 0; i < leftChannel.length; i++)
  //{
  //   let val = Math.floor(32767 * leftChannel[i]);
  //  val = Math.min(32767, val);
  //   val = Math.max(-32768, val);
  //   PCM16iSamples.push(val);
  //}
	for (let i = 0; i < leftChannel.length; i++)
  	{
    	 let tmp = Math.max(-1,Math.min(1,leftChannel[i]));
    	 tmp = tmp < 0 ? (tmp * 0x8000) : (tmp * 0x7FFF);
   	  	 tmp = tmp / 256;
    	 PCM16iSamples.push(tmp-256);
  	}
  	socket.emit('audioPacket',PCM16iSamples);
	console.log(PCM16iSamples);
  };
  // we connect the recorder
  volume.connect(recorder);
  // start recording
  recorder.connect(audioContext.destination);
  console.log('the mic is unmuted');
}
function mute(){
  if(recorder){
	  recorder.disconnect();
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