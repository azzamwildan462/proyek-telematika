import express from 'express';
import { fileURLToPath } from 'url';
import path, { dirname } from 'path';
import fs from 'fs';

const app = express();
const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

// Serve static files
const publicPath = path.join(__dirname, 'public');
app.use(express.static(publicPath));

// Middleware to parse JSON data in the request body
app.use(express.json());

// Define route handler for the root route ("/")
app.get('/', (req, res) => {
  res.sendFile(path.join(publicPath, 'script.js'));
});

// Define route handler for updating instructions
app.post('/append-instructions', (req, res) => {
  const instructions = req.body.instructions;
  const filePath = path.join(__dirname, 'public', 'instruksi.txt');

  fs.appendFile(filePath, instructions + '\n', (err) => {
    if (err) {
      console.error('Error updating instructions in the file:', err);
      res.status(500).send('Error updating instructions in the file');
    } else {
      console.log('Instructions updated in the file.');
      res.send('Instructions updated in the file');
    }
  });
});

// Start the server
app.listen(3000, () => {
  console.log('Server is running on port 3000');
});
