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
  let previousPosition = null;
  let instructions = []; // Store the instructions globally
  let utteranceIndex = 0; // Track the current utterance index

  // Initialize variables
  let currentInstructionIndex = 0; // Track the current instruction index

  // Function to speak the next instruction
  function speakNextInstruction() {
    if (synth.speaking) {
      synth.cancel(); // Stop speaking previous instructions
    }

    if (currentInstructionIndex < instructions.length) {
      const instruction = instructions[currentInstructionIndex];
      const utterance = utterances[currentInstructionIndex];

      utterance.onstart = () => {
        const distance = getDistanceFromInstruction(instruction);
        const instructionWithDistance = `${instruction} Jarak ${distance} meter.`;
        utterance.text = instructionWithDistance;
      };

      utterance.onend = () => {
        currentInstructionIndex++;
        speakNextInstruction(); // Call recursively to speak the next instruction
      };

      synth.speak(utterance);
    }
  }

  // Function to create utterances starting from a specific index
  function createUtterances(startIndex) {
    utterances = instructions.slice(startIndex).map(instruction => {
      const utterance = new SpeechSynthesisUtterance(instruction);
      utterance.lang = 'id-ID';
      return utterance;
    });
  }

  directions.on('route', function(event) {
    const routes = event.route;
    if (routes && routes.length > 0) {
      const steps = routes[0].legs[0].steps;
      instructions = steps.map(step => {
        const instruction = `${step.maneuver.instruction} jarak ${step.distance.toFixed(0)} meter.`;
        return instruction;
      });

      console.log('Instructions:', instructions);

      currentInstructionIndex = 0; // Reset the instruction index
      createUtterances(0); // Create the utterances array starting from index 0

      // Only send the instructions to the server once
      if (steps.length > 0 && steps[0].maneuver.type === 'depart') {
        sendInstructionsToServer(); // Send instructions to the server
      }

      speakNextInstruction(); // Speak the next instruction
    }
  });

  function createUtterances() {
    utterances = instructions.map(instruction => {
      const utterance = new SpeechSynthesisUtterance(instruction);
      utterance.lang = 'id-ID';
      return utterance;
    });
  }

  const inputButton = document.querySelector('.input-button-origin');
  inputButton.addEventListener('click', startSpeechRecognition2);

  function startSpeechRecognition2() {
    const recognition = new webkitSpeechRecognition();
    recognition.lang = 'id-ID';
    recognition.start();

    recognition.onresult = function(event) {
      const transcript = event.results[0][0].transcript;
      console.log('Transcript:', transcript);
      const destinationInput = document.querySelector('#mapbox-directions-destination-input');
      destinationInput.value = transcript; // set the value of the input field to the transcript
      directions.setDestination(transcript); // set the destination using the transcript
      // continue speech recognition for additional inputs
    };

    recognition.onend = function() {
      console.log('Voice recognition ended for destination input.');
      // Restart the speech recognition for destination input after a timeout
      startSpeechRecognition2();
    };
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
    const destinationInput = document.querySelector('#mapbox-directions-destination-input');

    if (!destinationInput.value) {
      destinationInput.value = transcript;
      directions.setDestination(transcript); // set the destination using the transcript
    }

    if (!destinationInput.value) {
      startSpeechRecognition2(); // continue speech recognition for unfilled inputs
    }
  };

  recognition.start();

  map.on('load', async () => {
    const coordinates = await getLocation();
    const { longitude, latitude } = coordinates;
  
    const geojson = {
      'type': 'FeatureCollection',
      'features': [
        {
          'type': 'Feature',
          'geometry': {
            'type': 'Point',
            'coordinates': [longitude, latitude]
          }
        }
      ]
    };
  
    map.addSource('location', {
      type: 'geojson',
      data: geojson
    });
  
    map.addLayer({
      'id': 'location',
      'type': 'symbol',
      'source': 'location',
      'layout': {
        'icon-image': 'rocket'
      }
    });
  
    const updateSource = setInterval(async () => {
      const coordinates = await getLocation(updateSource);
      geojson.features[0].geometry.coordinates = [coordinates.longitude, coordinates.latitude];
      map.getSource('location').setData(geojson);
  
      if (coordinates) {
        const { longitude, latitude } = coordinates;
  
        map.flyTo({ center: [longitude, latitude], zoom: 14 });
  
        const originInput = document.querySelector('#mapbox-directions-origin-input');
        originInput.value = `${longitude}, ${latitude}`;
  
        directions.setOrigin([longitude, latitude]);
      }
    }, 2000);
  
    async function getLocation(updateSource) {
      try {
        const response = await fetch('http://127.0.0.1:5000/data', { method: 'GET' });
        const { data } = await response.json();
        console.log(data);
  
        if (data.length > 0) {
          const { latitude, longitude } = data[0];
          return { longitude, latitude };
        }
  
        return null;
      } catch (err) {
        if (updateSource) clearInterval(updateSource);
        throw new Error(err);
      }
    }
  });
  

  // Function to get the distance from the instruction
  function getDistanceFromInstruction(instruction) {
    const regex = /jarak (\d+) meter/i;
    const match = instruction.match(regex);
    if (match && match[1]) {
      return match[1];
    }
    return 'Tidak diketahui';
  }

  // Function to send the instructions to the server
  function sendInstructionsToServer() {
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
        console.log('Instructions sent to server successfully.');
      } else {
        throw new Error('Failed to send instructions to server.');
      }
    })
    .catch(error => {
      console.log('Error:', error.message);
    });
  }
});