document.addEventListener('DOMContentLoaded', function() {
  mapboxgl.accessToken = 'pk.eyJ1IjoiaGVydHp6MiIsImEiOiJjbGloYW1ubXUweHJjM3FtcnpkbGlnZjJpIn0.Z4PV89xRLb8xx7e7F7cfYg';
  const map = new mapboxgl.Map({
    container: 'map',
    style: 'mapbox://styles/mapbox/streets-v12',
    center: [112.7964591992797, -7.284795818226529],
    zoom: 14
  });
  
  const directions = new MapboxDirections({
    accessToken: 'pk.eyJ1IjoiaGVydHp6MiIsImEiOiJjbGloYW1ubXUweHJjM3FtcnpkbGlnZjJpIn0.Z4PV89xRLb8xx7e7F7cfYg',
    unit: 'metric',
    language: 'id',
  });
  
  map.addControl(directions, 'top-left');
  const synth = window.speechSynthesis;
  let currentInputField = null;

  directions.on('route', function(event) {
    const instructions = event.route[0].legs[0].steps.map(step => {
      const instruction = `${step.maneuver.instruction}, jarak ${step.distance.toFixed(0)} meter.`;
      return instruction;
    });

    console.log('Instructions:', instructions);

    const utterances = instructions.map(instruction => {
      const utterance = new SpeechSynthesisUtterance(instruction);
      utterance.lang = 'id-ID';
      synth.speak(utterance);
    });
    
    const instructionsData = {
      instructions: instructions.join('\n') // Join the instructions with line breaks
    };

    fetch('/append-instructions', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(instructionsData)
    })
    .then(response => {
      if (response.ok) {
        console.log('Instructions sent to the server.');
      } else {
        console.error('Error sending instructions to the server:', response.status);
      }
    })
    .catch(error => {
      console.error('Error sending instructions to the server:', error);
    });
  });

  const inputButton = document.querySelector('.input-button-origin');
  inputButton.addEventListener('click', startSpeechRecognition);

  const inputButton2 = document.querySelector('.input-button-destination');
  inputButton2.addEventListener('click', startSpeechRecognition2);

  function startSpeechRecognition() {
    const recognition = new webkitSpeechRecognition();
    recognition.lang = 'id-ID';
    recognition.start();

    recognition.onresult = function(event) {
      const transcript = event.results[0][0].transcript;
      console.log('Transcript:', transcript);
      if (currentInputField) {
        currentInputField.value = transcript; // set the value of the input field to the transcript
        if (currentInputField.id === 'mapbox-directions-origin-input') {
          directions.setOrigin(transcript); // set the origin using the transcript
          currentInputField = document.querySelector('#mapbox-directions-destination-input'); // set the current input field to the destination input field
          startSpeechRecognition(); // start speech recognition for the destination input field
        } else if (currentInputField.id === 'mapbox-directions-destination-input') {
          directions.setDestination(transcript); // set the destination using the transcript
          startSpeechRecognition(); // continue speech recognition for additional inputs
        }
      }
    }
  }
  

  function startSpeechRecognition2() {
    const recognition = new webkitSpeechRecognition();
    recognition.lang = 'id-ID';
    recognition.start();

    recognition.onresult = function(event) {
      const DestinationInput = document.querySelector('#mapbox-directions-destination-input');
      const transcript = event.results[0][0].transcript;
      console.log('Transcript:', transcript);
      DestinationInput.value = transcript; // set the value of the input field to the transcript
      directions.setDestination(transcript); // set the origin using the transcript
       // continue speech recognition for additional inputs
    }
    recognition.onend = function() {
      console.log('Voice recognition ended for destination input.');
      // Restart the speech recognition for destination input after a timeout
      startSpeechRecognition2();
    }
  }

  navigator.permissions.query({ name: 'microphone' })
    .then(function(permissionStatus) {
      if (permissionStatus.state === 'granted') {
        inputButton.disabled = false;
      } else {
        permissionStatus.onchange = function() {
          console.log('Microphone permission status:', permissionStatus.state);
          if (permissionStatus.state === 'granted') {
            inputButton.disabled = false;
          }
        };
      }
    });

  const recognition = new webkitSpeechRecognition();
  recognition.lang = 'id-ID';

  recognition.onresult = function(event) {
    const transcript = event.results[0][0].transcript.toLowerCase();
    console.log('Transcript:', transcript);
    const originInput = document.querySelector('#mapbox-directions-origin-input');
    const destinationInput = document.querySelector('#mapbox-directions-destination-input');

    if (!originInput.value) {
      originInput.value = transcript;
      currentInputField = originInput;
    } else if (!destinationInput.value) {
      destinationInput.value = transcript;
      currentInputField = destinationInput;
    }

    if (!originInput.value || !destinationInput.value) {
      startSpeechRecognition(); // continue speech recognition for unfilled inputs
    }
  }

  recognition.start();
});
