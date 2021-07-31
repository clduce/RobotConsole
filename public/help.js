//Provides help messages for widgets and other things

function showHelp(){
	document.getElementById('configWindow').style.left = '48%';
	document.getElementById('helpWindow').style.display = 'inline';
}
function hideHelp(){
	document.getElementById('configWindow').style.left = '25%';
	document.getElementById('helpWindow').style.display = 'none';
}
function toggleHelp(){
	if(document.getElementById('helpWindow').style.display == 'none'){
		showHelp();
	}
	else{
		hideHelp();
	}
}

const helpMessages = {}
loadhelpMessagesFromText();

//returns string as pagargraph element
function toParagraph(s){
	return `<p>${s}</p>`;
}

function updateHelpWindow(type){
	document.getElementById('helpArea').innerHTML = helpMessages[type] || '';
}

var helpIsCalled = false;
function loadhelpMessagesFromText(){
	if(helpIsCalled) return;
	helpIsCalled = true;
	var xhttp = new XMLHttpRequest();
	xhttp.onreadystatechange = function(){
		if(this.readyState == 4 && this.status == 200){
			let lines = xhttp.responseText.split('\n');
			cvtLinesToHelp(lines);
		}
	}
	xhttp.open("GET","widgetHelp.txt",true);
	xhttp.send();
}


function cvtLinesToHelp(lines){
	let currentWidget = '';
	let currentMessage = '';
	for(let i = 0; i < lines.length; i++){
		if(lines[i][0] == '_'){
			console.log('widget',lines[i]);
			if(currentWidget){
				helpMessages[currentWidget] = toParagraph(currentMessage.trimRight('<br>'));
				console.log(helpMessages[currentWidget]);
			}
			currentWidget = lines[i];
			currentMessage = '';
		}else{
			currentMessage += lines[i]+'<br>';
			console.log('message',lines[i]);
		}
	}
	if(currentWidget){
		helpMessages[currentWidget] = toParagraph(currentMessage.trimRight('<br>'));
		helpMessages[currentWidget].replace('\n','<br>');
	}
}