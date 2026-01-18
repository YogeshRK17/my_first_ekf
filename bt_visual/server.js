const express = require("express");
const app = express();

const PORT = 8080;           // change if needed
const HOST = "0.0.0.0";      // listen on all interfaces

app.use(express.static("public"));

app.listen(PORT, HOST, () => {
  console.log(`Behavior Tree Visualizer running at http://${HOST}:${PORT}`);
});
