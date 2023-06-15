var map;
var directions;

var directions = new MapboxDirections({
  accessToken: 'pk.eyJ1IjoiaGVydHp6MiIsImEiOiJjbGloYW1ubXUweHJjM3FtcnpkbGlnZjJpIn0.Z4PV89xRLb8xx7e7F7cfYg',
  unit: 'metric',
  profile:'mapbox/walking'
});

var map = new mapboxgl.Map({
  container:'map',
  style: 'mapbox://styles/mapbox/streets-v12',
  center: [112.7964591992797, -7.284795818226529],
  zoom: 14,
});

map.addControl(directions, 'top-left');

let routeInstructions = []; // Store the route instructions

directions.on('route', function (event) {
  routeInstructions = event.route[0].legs[0].steps.map((step) => {
    const instruction = `${step.maneuver.instruction}, jarak ${step.distance.toFixed(0)} meter.`;
    return instruction;
  });

  console.log('Instructions:', routeInstructions);

  const synthesizer = new TextToSpeech();
  const utterances = routeInstructions.map((instruction) => {
    synthesizer.speak(instruction, 'id-ID');
  });
});

function startSpeechRecognition(transcript) {
  // Handle speech recognition logic here
  // Replace currentInputField.value = transcript; with appropriate server-side logic
  // Replace directions.setOrigin(transcript); and directions.setDestination(transcript); with appropriate server-side logic

  // Send the route instructions as a response
  return routeInstructions;
}

// Add server-side logic for handling HTTP requests
// For example, using Express.js:
const express = require('express');
const app = express();

app.use(express.json());

app.post('/api/start-speech-recognition', (req, res) => {
  const { transcript } = req.body;
  const instructions = startSpeechRecognition(transcript);
  res.json({ status: 'success', instructions });
});

// Start the server
app.listen(3000, () => {
  console.log('Server is running on port 3000');
});
