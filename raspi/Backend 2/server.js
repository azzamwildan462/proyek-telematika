import express from 'express';
import { fileURLToPath } from 'url';
import path, { dirname } from 'path';
import fs from 'fs';
import { exec } from 'child_process';

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
  const filePath = path.join(publicPath, 'instruksi.txt');
  fs.unlinkSync(filePath);
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
/*
// Define route handler for executing PHP files
app.get('/execute-php', (req, res) => {
  const phpFilePath = path.join(publicPath, 'get_location.php');

  exec(`"${path.join('C:', 'xampp', 'php', 'php.exe')}" ${phpFilePath}`, (error, stdout, stderr) => {
    if (error) {
      console.error('Error executing PHP file:', error);
      res.status(500).send('Error executing PHP file');
      return;
    }

    if (stderr) {
      console.error('PHP Error output:', stderr);
      res.status(500).send('PHP Error output');
      return;
    }

    console.log('PHP Output:', stdout);
    res.send('PHP file executed successfully');
  });
});
*/
// Start the server
app.listen(3000, () => {
  console.log('Server is running on port 3000');
});