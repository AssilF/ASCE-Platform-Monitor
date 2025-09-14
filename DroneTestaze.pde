import processing.serial.*;
import peasy.*;
import controlP5.*;
import java.net.*;
import java.io.*;
import java.util.Arrays;
import ddf.minim.*;
import ddf.minim.analysis.*;

Slider pitchKp, pitchKi, pitchKd;
Slider rollKp, rollKi, rollKd;
Slider yawKp, yawKi, yawKd;
Slider pitchFilterFreq, pitchFilterQ;
Slider rollFilterFreq, rollFilterQ;
Slider yawFilterFreq, yawFilterQ;
Button resetButton, armButton, disarmButton, sendFilters;


Minim minim;
FFT fft;
float sampleRate = 100.0; // 100 Hz IMU update rate

AudioInput input;
float[] fftInput = new float[512];
int fftIndex = 0;
String[] fftSources = {"Pitch", "Roll", "Yaw", "PitchCorr", "RollCorr", "YawCorr"};
int selectedFFTIndex = 0;

// Communication
Socket socket;
PrintWriter out;
BufferedReader in;
processing.serial.Serial myPort;
boolean isConnected = false;
Thread tcpReaderThread;
boolean keepReading = true;
boolean[] graphEnabled = {true, true, true, false, false, false};

// UI
ControlP5 cp5;
PeasyCam cam;

// PID Constants
float kp_pitch = 2.0, ki_pitch = 0.0, kd_pitch = 0.5;
float kp_roll = 2.0, ki_roll = 0.0, kd_roll = 0.5;
float kp_yaw = 2.0, ki_yaw = 0.0, kd_yaw = 0.5;

// 3D Model
PShape droneModel;

// Drone telemetry
float pitch = 0;
float roll = 0;
float yaw = 0;
float pitchCorr = 0;
float rollCorr = 0;
float yawCorr = 0;
int throttle = 1000;
float pitchBias = 0;
float rollBias = 0;
float yawBias = 0;
int lastCmdTime = 0;

// PID graphs
int graphLength = 300;
float[] pitchCorrGraph = new float[graphLength];
float[] rollCorrGraph = new float[graphLength];
float[] yawCorrGraph = new float[graphLength];
float[] pitchGraph = new float[graphLength];
float[] rollGraph = new float[graphLength];
float[] yawGraph = new float[graphLength];

// Graph parameters
float graphYMin = -50;
float graphYMax = 50;
int graphUpdateRate = 1; // Update every N frames
int graphFrameCounter = 0;

// Serial messages
ArrayList<String> serialMessages = new ArrayList<String>();
int maxMessages = 16;

// Connection status
String connectionStatus = "Disconnected";

void setup() {
  size(1200, 800, P3D);
  
  // Initialize UI
  setupUI();
  
  // Initialize audio
  minim = new Minim(this);
  input = minim.getLineIn(Minim.MONO, 512);
  fft = new FFT(512, sampleRate);


  // Load 3D model
  loadDroneModel();
  
  // Initialize camera
  cam = new PeasyCam(this, 0, 0, 0, 600);
  
  // Setup communication
  setupCommunication();
  
  // Initialize graphs
  initializeGraphs();
  
  // Window positioning
  surface.setLocation(100, 100);
  surface.setSize(1200, 800);
}

void setupUI() {
  cp5 = new ControlP5(this);
  cp5.setAutoDraw(false);
  
  // Pitch Controls
  cp5.addTextfield("Kp_pitch").setPosition(50, 40).setSize(80, 25).setText(str(kp_pitch));
  cp5.addTextfield("Ki_pitch").setPosition(50, 70).setSize(80, 25).setText(str(ki_pitch));
  cp5.addTextfield("Kd_pitch").setPosition(50, 100).setSize(80, 25).setText(str(kd_pitch));
  
  // Roll Controls
  cp5.addTextfield("Kp_roll").setPosition(140, 40).setSize(80, 25).setText(str(kp_roll));
  cp5.addTextfield("Ki_roll").setPosition(140, 70).setSize(80, 25).setText(str(ki_roll));
  cp5.addTextfield("Kd_roll").setPosition(140, 100).setSize(80, 25).setText(str(kd_roll));
  
  // Yaw Controls
  cp5.addTextfield("Kp_yaw").setPosition(230, 40).setSize(80, 25).setText(str(kp_yaw));
  cp5.addTextfield("Ki_yaw").setPosition(230, 70).setSize(80, 25).setText(str(ki_yaw));
  cp5.addTextfield("Kd_yaw").setPosition(230, 100).setSize(80, 25).setText(str(kd_yaw));
  

  // Buttons
  cp5.addButton("Send").setPosition(50, 140).setSize(60, 30);
  cp5.addButton("Save").setPosition(120, 140).setSize(60, 30);
  cp5.addButton("Status").setPosition(190, 140).setSize(60, 30);
  cp5.addButton("Connect").setPosition(260, 140).setSize(60, 30);
  cp5.addButton("Reset").setPosition(50, 175).setSize(60, 30);
  armButton = cp5.addButton("Arm").setPosition(120, 175).setSize(60, 30);
  disarmButton = cp5.addButton("Disarm").setPosition(190, 175).setSize(60, 30);
  sendFilters =cp5.addButton("Send Filters").setPosition(260, 175).setSize(60, 30);
  
  // Graph Toggle Checkboxes
  cp5.addCheckBox("GraphToggles")
     .setPosition(1120, 520)
     .setSize(10, 10)
     .setItemsPerRow(1)
     .addItem("Pitch", 0)
     .addItem("Roll", 1)
     .addItem("Yaw", 2)
     .addItem("PitchCorr", 3)
     .addItem("RollCorr", 4)
     .addItem("YawCorr", 5)
     .activate(0)  // Pitch
     .activate(1)  // Roll
     .activate(2); // Yaw

  cp5.addScrollableList("FFTSource")
     .setPosition(1015, 520)
     .setSize(100, 100)
     .setBarHeight(20)
     .setItemHeight(20)
     .addItems(Arrays.asList(fftSources))
     .setValue(0); // Default to Pitch
  
  // Command line
  cp5.addTextfield("CommandLine").setPosition(50, 470).setSize(300, 30).setText("").setAutoClear(false);
  cp5.addButton("SendCommand").setPosition(360, 470).setSize(80, 30);
  
  // Graph parameters
  cp5.addTextfield("GraphYMin").setPosition(50, 520).setSize(80, 25).setText(str(graphYMin));
  cp5.addTextfield("GraphYMax").setPosition(140, 520).setSize(80, 25).setText(str(graphYMax));
  cp5.addTextfield("GraphLength").setPosition(230, 520).setSize(80, 25).setText(str(graphLength));
  cp5.addTextfield("GraphUpdateRate").setPosition(320, 520).setSize(80, 25).setText(str(graphUpdateRate));
  cp5.addButton("UpdateGraph").setPosition(410, 520).setSize(80, 25);
 

      
  pitchFilterFreq = cp5.addSlider("Pitch Filter Freq").setPosition(50, 570).setRange(5, 100).setValue(10);
  pitchFilterQ = cp5.addSlider("Pitch Filter Q").setPosition(50, 580).setRange(0.1f, 10.0f).setValue(0.707);

  rollFilterFreq = cp5.addSlider("Roll Filter Freq").setPosition(230, 570).setRange(5, 100).setValue(10);
  rollFilterQ = cp5.addSlider("Roll Filter Q").setPosition(230, 580).setRange(0.1f, 10.0f).setValue(0.707);

  yawFilterFreq = cp5.addSlider("Yaw Filter Freq").setPosition(405, 570).setRange(5, 100).setValue(10);
  yawFilterQ = cp5.addSlider("Yaw Filter Q").setPosition(405, 580).setRange(0.1f, 10.0f).setValue(0.707);
  
}

void loadDroneModel() {
  try {
    droneModel = loadShape("drone.obj");
    if (droneModel != null) {
      droneModel.scale(-1);
    }
  } catch (Exception e) {
    println("⚠️ Could not load drone.obj, using basic shape");
    droneModel = null;
  }
}

void FFTSource(int index) {
  selectedFFTIndex = (int) index;
}

void setupCommunication() {
  // Try Serial first
  String[] ports = processing.serial.Serial.list();
  println("Available serial ports:", ports);
  
  if (ports.length > 0) {
    try {
      myPort = new processing.serial.Serial(this, ports[0], 115200);
      myPort.bufferUntil('\n');
      println("Connected to serial:", ports[0]);
      isConnected = true;
      connectionStatus = "Serial: " + ports[0];
      addSerialMessage("Connected to serial: " + ports[0]);
      return;
    } catch (Exception e) {
      println("Failed to open serial port:", e);
      myPort = null;
    }
  } else {
    println("⚠️ No serial port detected.");
  }
  
  // Try WiFi connection if serial fails
  tryWiFiConnection();
}

void tryWiFiConnection() {
  try {
    println("Attempting TCP connection to 192.168.4.1:8000...");
    addSerialMessage("Attempting TCP connection...");
    
    // Create socket with timeout
    socket = new Socket();
    socket.connect(new InetSocketAddress("192.168.4.1", 8000), 5000); // 5 second timeout
    socket.setSoTimeout(1000); // 1 second read timeout
    
    // Setup streams
    out = new PrintWriter(socket.getOutputStream(), true);
    in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
    
    println("Connected to ESP32 over TCP!");
    isConnected = true;
    connectionStatus = "TCP: 192.168.4.1:8000";
    addSerialMessage("Connected to ESP32 over TCP!");
    
    // Start background thread to read incoming data
    startTCPReaderThread();
    
  } catch (Exception e) {
    println("⚠️ Failed to connect over TCP:", e.getMessage());
    socket = null;
    out = null;
    in = null;
    isConnected = false;
    connectionStatus = "TCP Connection Failed";
    addSerialMessage("TCP Connection Failed: " + e.getMessage());
  }
}

void startTCPReaderThread() {
  keepReading = true;
  tcpReaderThread = new Thread(new Runnable() {
    public void run() {
      while (keepReading && socket != null && !socket.isClosed()) {
        try {
          if (in != null && in.ready()) {
            String line = in.readLine();
            if (line != null) {
              // Process the line in the main thread
              handleIncomingLine(line.trim());
            }
          }
          Thread.sleep(10); // Small delay to prevent excessive CPU usage
        } catch (SocketTimeoutException e) {
          // Normal timeout, continue reading
        } catch (Exception e) {
          println("TCP read error:", e.getMessage());
          // Connection lost, mark as disconnected
          isConnected = false;
          connectionStatus = "TCP Connection Lost";
          addSerialMessage("TCP Connection Lost: " + e.getMessage());
          break;
        }
      }
      println("TCP reader thread stopped");
    }
  });
  tcpReaderThread.start();
}

void initializeGraphs() {
  for (int i = 0; i < graphLength; i++) {
    pitchCorrGraph[i] = 0;
    rollCorrGraph[i] = 0;
    yawCorrGraph[i] = 0;
    pitchGraph[i] = 0;
    rollGraph[i] = 0;
    yawGraph[i] = 0;
  }
}

void addSerialMessage(String message) {
  serialMessages.add(message);
  if (serialMessages.size() > maxMessages) {
    serialMessages.remove(0);
  }
}

void draw() {
  background(30);
  lights();
  
  // Update FFT with selected data

  updateFFT();
  
  // 3D Drone visualization
  draw3DDrone();
  
  // 2D GUI overlay
  draw2DGUI();
  

}

void draw3DDrone() {
  // Draw origin axes
  drawOriginAxes();
  
  // Draw thrust arrow first in world coordinates (always points up)
  drawThrustArrow();
  
  // Then draw the drone with its rotations
  pushMatrix();
  rotateY(radians(yaw));
  rotateX(radians(roll));
  rotateZ(radians(-pitch));
  
  if (droneModel != null) {
    shape(droneModel);
  } else {
    drawBasicDrone();
  }
  
  drawPitchRollYawArrows();
  popMatrix();
}

void drawOriginAxes() {
  strokeWeight(3);
  
  // X-axis (Red)
  stroke(255, 0, 0);
  line(0, 0, 0, 100, 0, 0);
  
  // Y-axis (Green)
  stroke(0, 255, 0);
  line(0, 0, 0, 0, 0, 100);
  
  // Z-axis (Blue)
  stroke(0, 0, 255);
  line(0, 0, 0, 0, -100, 0);
  
  // Axis labels
  textSize(12);
  fill(255, 0, 0);
  text("X", 105, 5, 0);
  fill(0, 255, 0);
  text("Y", 5, 6, 105);
  fill(0, 0, 255);
  text("Z", 8, -95, 5);
}

void draw2DGUI() {
  hint(DISABLE_DEPTH_TEST);
  cam.beginHUD();
  
  // Connection status
  fill(isConnected ? color(0, 255, 0) : color(255, 0, 0));
  text("Status: " + connectionStatus, 400, 42);
  
  // Telemetry display
  fill(255);
  text("Pitch PID", 49-30, 29);
  text("Roll PID", 139-30, 29);
  text("Yaw PID", 229-30, 29);
  text("Telemetry:", 400-30, 60);
  text("Pitch: " + nf(pitch, 0, 2) + "°", 400-30, 80);
  text("Roll: " + nf(roll, 0, 2) + "°", 400-30, 100);
  text("Yaw: " + nf(yaw, 0, 2) + "°", 400-30, 120);
  text("Throttle: " + throttle, 400-30, 140);
  text("Pitch Corr: " + nf(pitchCorr, 0, 2), 400-30, 160);
  text("Roll Corr: " + nf(rollCorr, 0, 2), 400-30, 180);
  text("Yaw Corr: " + nf(yawCorr, 0, 2), 400-30, 200);
  text("Pitch Bias: " + nf(pitchBias, 0, 2), 400-30, 220);
  text("Roll Bias: " + nf(rollBias, 0, 2), 400-30, 240);
  text("Yaw Bias: " + nf(yawBias, 0, 2), 400-30, 260);
  text("Last Cmd: " + lastCmdTime + "ms", 400-30, 280);
  
  // Command line label
  fill(255);
  //text("Command Line:", 50, 460);
  
  // Graph parameters labels
  //text("Graph Settings:", 49, 513);
  //text("Y Min", 50, 513);
  //text("Y Max", 140, 513);
  //text("Length", 230, 513);
  //text("Update Rate", 322, 513);
  
  // Serial messages box
  drawSerialMessages();
  
  cp5.draw();
  fft.forward(input.mix);
  drawGraphs();
  drawFFT(780, 20, 400, 150);  // X, Y, W, H
  cam.endHUD();
  hint(ENABLE_DEPTH_TEST);
}

void drawSerialMessages() {
  // Serial messages box
  fill(0, 100);
  stroke(100);
  rect(50, 220, 300, 240);
  
  fill(255);
  text("Serial Messages:", 60, 240);
  
  // Display messages
  textSize(10);
  for (int i = 0; i < serialMessages.size(); i++) {
    String msg = serialMessages.get(i);
    if (msg.length() > 2 && msg.substring(0,2).equals("DB:")) {
      break;
    }else
   if (msg.length() > 64) {
      msg = msg.substring(0, 64) + "...";
    }
    text(msg, 54, 254 + i * 12);
  }
  textSize(12); // Reset text size
}

void drawBasicDrone() {
  // Body: vertical cylinder
  fill(100, 200, 255);
  noStroke();
  pushMatrix();
  rotateX(HALF_PI);
  cylinder(20, 60);
  popMatrix();
  
  // Arms: X and Y lines
  stroke(255);
  strokeWeight(4);
  line(-60, 0, 0, 60, 0, 0);
  line(0, -60, 0, 0, 60, 0);
  
  // Props
  fill(255, 100, 100);
  noStroke();
  pushMatrix();
  translate(-60, 0, 0);
  rotateZ(HALF_PI);
  cylinder(2, 30);
  popMatrix();
  
  pushMatrix();
  translate(60, 0, 0);
  rotateZ(HALF_PI);
  cylinder(2, 30);
  popMatrix();
  
  pushMatrix();
  translate(0, -60, 0);
  cylinder(2, 30);
  popMatrix();
  
  pushMatrix();
  translate(0, 60, 0);
  cylinder(2, 30);
  popMatrix();
}

void drawThrustArrow() {
  float arrowLength = map(throttle, 1000, 2000, 50, 300);
  stroke(255, 0, 0);
  strokeWeight(6);
  
  // Draw thrust arrow pointing up in world coordinates (always vertical)
  line(0, 0, 0, 0, -arrowLength, 0);
  
  // Draw arrowhead
  pushMatrix();
  translate(0, -arrowLength-20, 0);
  rotateX(radians(0));
  rotateY(radians(180));
  fill(255, 0, 0);
  noStroke();
  cone(10, 20);
  popMatrix();
}

void drawPitchRollYawArrows() {
  // Draw pitch correction arrow (around X-axis)
  if (abs(pitchCorr) > 0.1) {
    drawCorrectionArrow(pitchCorr, 0, 255, 255, 0); // Green, around X-axis
  }
  
  // Draw roll correction arrow (around Y-axis) 
  if (abs(rollCorr) > 0.1) {
    drawCorrectionArrow(rollCorr, 0, 255, 81, 1); // Cyan, around Y-axis
  }
  
  // Draw yaw correction arrow (around Z-axis)
  if (abs(yawCorr) > 0.1) {
    drawCorrectionArrow(yawCorr, 255, 0, 255, 2); // Magenta, around Z-axis
  }
}

void drawCorrectionArrow(float correction, int r, int g, int b, int axis) {
  pushMatrix();
  
  // Position the arrow away from center to avoid overlap
  if (axis == 0) { // Pitch (X-axis)
    translate(0, -80, 0);
  } else if (axis == 1) { // Roll (Y-axis)
    translate(-80, 0, 0);
  } else { // Yaw (Z-axis)
    translate(0, 0, 80);
  }
  
  stroke(r, g, b);
  strokeWeight(3);
  noFill();
  
  // Map correction value to arc angle (max 180 degrees)
  float maxAngle = PI; // 180 degrees
  float angle = map(abs(correction), 0, 50, 0, maxAngle);
  angle = constrain(angle, 0, maxAngle);
  
  // Draw arc
  int segments = 20;
  float radius = 40;
  
  beginShape();
  for (int i = 0; i <= segments; i++) {
    float t = map(i, 0, segments, 0, angle);
    float x, y, z;
    
    if (axis == 0) { // Pitch correction around X-axis
      x = 0;
      y = radius * cos(t);
      z = radius * sin(t);
      if (correction < 0) z = -z; // Flip direction for negative correction
    } else if (axis == 1) { // Roll correction around Y-axis
      x = radius * cos(t);
      y = 0;
      z = radius * sin(t);
      if (correction < 0) z = -z; // Flip direction for negative correction
    } else { // Yaw correction around Z-axis
      x = radius * cos(t);
      y = radius * sin(t);
      z = 0;
      if (correction < 0) y = -y; // Flip direction for negative correction
    }
    
    vertex(x, y, z);
  }
  endShape();
  
  // Draw arrowhead at the end of the arc
  if (angle > 0.1) {
    float t = angle;
    float x, y, z;
    
    if (axis == 0) { // Pitch
      x = 0;
      y = radius * cos(t);
      z = radius * sin(t);
      if (correction < 0) z = -z;
    } else if (axis == 1) { // Roll
      x = radius * cos(t);
      y = 0;
      z = radius * sin(t);
      if (correction < 0) z = -z;
    } else { // Yaw
      x = radius * cos(t);
      y = radius * sin(t);
      z = 0;
      if (correction < 0) y = -y;
    }
    
    // Draw small arrowhead
    pushMatrix();
    translate(x, y, z);
    fill(r, g, b);
    noStroke();
    sphere(3);
    popMatrix();
  }
  
  popMatrix();
}

void shiftLeft(float[] array) {
  for (int i = 1; i < array.length; i++) {
    array[i - 1] = array[i];
  }
}

void drawGraphs() {
  pushMatrix();
  translate(0, height - 200);
  
  // Graph background
  fill(0, 150);
  noStroke();
  rect(0, 0, width, 200);
  
  // Grid lines
  stroke(50);
  strokeWeight(1);
  for (int i = 0; i <= 4; i++) {
    float y = map(i, 0, 4, 180, 20);
    line(0, y, width, y);
  }
  
  // Center line
  stroke(100);
  float centerY = map(0, graphYMin, graphYMax, 180, 20);
  line(0, centerY, width, centerY);
  
  // Update graph data at specified rate
  if (graphFrameCounter++ >= graphUpdateRate) {
    graphFrameCounter = 0;
    updateGraphData();
  }
  
  // Draw graphs based on enabled state
  if (graphEnabled[0]) { // Pitch
    drawGraph(pitchGraph, 255, 0, 0); // Red
  }
  if (graphEnabled[1]) { // Roll
    drawGraph(rollGraph, 0, 255, 0); // Green
  }
  if (graphEnabled[2]) { // Yaw
    drawGraph(yawGraph, 0, 0, 255); // Blue
  }
  if (graphEnabled[3]) { // Pitch Correction
    drawGraph(pitchCorrGraph, 255, 128, 0); // Orange
  }
  if (graphEnabled[4]) { // Roll Correction
    drawGraph(rollCorrGraph, 0, 255, 255); // Cyan
  }
  if (graphEnabled[5]) { // Yaw Correction
    drawGraph(yawCorrGraph, 255, 0, 255); // Magenta
  }
  
  // Labels
  fill(255);
  text("Drone Telemetry Graphs  ---  RED:pitch  |  GREEN:roll  |  BLUE:yaw  |  ORANGE:pitchPID  |  CYAN:rollPID  |  MAGENTA:yawPID", 10, 15);
  text("Range: " + graphYMin + " to " + graphYMax + " | Length: " + graphLength + " | Update Rate: " + graphUpdateRate, 10, 195);
  
  popMatrix();
}

void drawGraph(float[] data, int r, int g, int b) {
  stroke(r, g, b);
  strokeWeight(2);
  noFill();
  beginShape();
  for (int i = 0; i < graphLength; i++) {
    float y = map(data[i], graphYMin, graphYMax, 180, 20);
    vertex(i * (width / (float)graphLength), y);
  }
  endShape();
}

void updateGraphData() {
  // Shift arrays left and add new data
  shiftLeft(pitchGraph);
  shiftLeft(rollGraph);
  shiftLeft(yawGraph);
  shiftLeft(pitchCorrGraph);
  shiftLeft(rollCorrGraph);
  shiftLeft(yawCorrGraph);
  
  // Add new data points
  pitchGraph[graphLength - 1] = pitch;
  rollGraph[graphLength - 1] = roll;
  yawGraph[graphLength - 1] = yaw;
  pitchCorrGraph[graphLength - 1] = pitchCorr;
  rollCorrGraph[graphLength - 1] = rollCorr;
  yawCorrGraph[graphLength - 1] = yawCorr;
}

// Communication handlers
void serialEvent(processing.serial.Serial port) {
  String line = port.readStringUntil('\n');
  if (line != null) {
    handleIncomingLine(line.trim());
  }
}

void handleIncomingLine(String line) {
  if (line.isEmpty()) return;
  
  println("ESP ->", line);
  if(!line.startsWith("DB:")){
  addSerialMessage("ESP -> " + line);
  }
  
  if (line.startsWith("DB:")) {
    parseTelemetryData(line);
  } else if (line.startsWith("Pitch PID:")) {
    parsePitchPID(line);
  } else if (line.startsWith("Roll  PID:")) {
    parseRollPID(line);
  } else if (line.startsWith("Yaw   PID:")) {
    parseYawPID(line);
  }else if (line.startsWith("PITCH_FILTER")) {
  parsePitchFilter(line);
} else if (line.startsWith("ROLL_FILTER")) {
  parseRollFilter(line);
} else if (line.startsWith("YAW_FILTER")) {
  parseYawFilter(line);
}
}

void parseTelemetryData(String line) {
  // New format: DB:pitch roll yaw pitchCorr rollCorr yawCorr throttle pitchBias rollBias yawBias lastCmdTime
  line = line.substring(3).trim();
  String[] parts = split(line, ' ');
  if (parts.length >= 11) {
    try {
      pitch = float(parts[0]);
      roll = float(parts[1]);
      yaw = float(parts[2]);
      pitchCorr = float(parts[3]);
      rollCorr = float(parts[4]);
      yawCorr = float(parts[5]);
      throttle = int(parts[6]);
      pitchBias = float(parts[7]);
      rollBias = float(parts[8]);
      yawBias = float(parts[9]);
      lastCmdTime = int(parts[10]);
    } catch (NumberFormatException e) {
      println("Parse error in telemetry: " + e.getMessage());
    }
  }
}

void parsePitchFilter(String line) {
  String[] tokens = splitTokens(line, " ");
  if (tokens.length >= 3) {
    try {
      float q = parseFloat(tokens[1]);
      float freq = parseFloat(tokens[2]);
      cp5.getController("Pitch Filter Q").setValue(q);
      cp5.getController("Pitch Filter Freq").setValue(freq);
    } catch (Exception e) {
      println("Failed to parse Pitch Filter: " + e.getMessage());
    }
  }
}

void parseRollFilter(String line) {
  String[] tokens = splitTokens(line, " ");
  if (tokens.length >= 3) {
    try {
      float q = parseFloat(tokens[1]);
      float freq = parseFloat(tokens[2]);
      cp5.getController("Roll Filter Q").setValue(q);
      cp5.getController("Roll Filter Freq").setValue(freq);
    } catch (Exception e) {
      println("Failed to parse Roll Filter: " + e.getMessage());
    }
  }
}

void parseYawFilter(String line) {
  String[] tokens = splitTokens(line, " ");
  if (tokens.length >= 3) {
    try {
      float q = parseFloat(tokens[1]);
      float freq = parseFloat(tokens[2]);
      cp5.getController("Yaw Filter Q").setValue(q);
      cp5.getController("Yaw Filter Freq").setValue(freq);
    } catch (Exception e) {
      println("Failed to parse Yaw Filter: " + e.getMessage());
    }
  }
}

void parsePitchPID(String line) {
  String[] tokens = match(line, "Kp=([0-9.]+)\\s+Ki=([0-9.]+)\\s+Kd=([0-9.]+)");
  if (tokens != null && tokens.length >= 4) {
    try {
      kp_pitch = float(tokens[1]);
      ki_pitch = float(tokens[2]);
      kd_pitch = float(tokens[3]);
      
      cp5.get(Textfield.class, "Kp_pitch").setText(str(kp_pitch));
      cp5.get(Textfield.class, "Ki_pitch").setText(str(ki_pitch));
      cp5.get(Textfield.class, "Kd_pitch").setText(str(kd_pitch));
    } catch (NumberFormatException e) {
      println("Parse error in pitch PID: " + e.getMessage());
    }
  }
}

void parseRollPID(String line) {
  String[] tokens = match(line, "Kp=([0-9.]+)\\s+Ki=([0-9.]+)\\s+Kd=([0-9.]+)");
  if (tokens != null && tokens.length >= 4) {
    try {
      kp_roll = float(tokens[1]);
      ki_roll = float(tokens[2]);
      kd_roll = float(tokens[3]);
      
      cp5.get(Textfield.class, "Kp_roll").setText(str(kp_roll));
      cp5.get(Textfield.class, "Ki_roll").setText(str(ki_roll));
      cp5.get(Textfield.class, "Kd_roll").setText(str(kd_roll));
    } catch (NumberFormatException e) {
      println("Parse error in roll PID: " + e.getMessage());
    }
  }
}

// Missing functions and completion for DroneTestaze.pde

// Complete the parseYawPID function
void parseYawPID(String line) {
  String[] tokens = match(line, "Kp=([0-9.]+)\\s+Ki=([0-9.]+)\\s+Kd=([0-9.]+)");
  if (tokens != null && tokens.length >= 4) {
    try {
      kp_yaw = float(tokens[1]);
      ki_yaw = float(tokens[2]);
      kd_yaw = float(tokens[3]);
      
      cp5.get(Textfield.class, "Kp_yaw").setText(str(kp_yaw));
      cp5.get(Textfield.class, "Ki_yaw").setText(str(ki_yaw));
      cp5.get(Textfield.class, "Kd_yaw").setText(str(kd_yaw));
    } catch (NumberFormatException e) {
      println("Parse error in yaw PID: " + e.getMessage());
    }
  }
}

void controlEvent(ControlEvent e) {
  String name = e.getName();
  if (name == null) return;

  if (name.equals("Reset")) {
    sendCommand("reset");
  } else if (name.equals("Arm")) {
    sendCommand("arm");
  } else if (name.equals("Disarm")) {
    sendCommand("disarm");
  } else if (name.equals("Send Filters")) {
    sendCommand("setfilter pitch freq " + pitchFilterFreq.getValue());
    delay(30);
    sendCommand("setfilter pitch q " + pitchFilterQ.getValue());
    delay(30);
    sendCommand("setfilter roll freq " + rollFilterFreq.getValue());
    delay(30);
    sendCommand("setfilter roll q " + rollFilterQ.getValue());
    delay(30);
    sendCommand("setfilter yaw freq " + yawFilterFreq.getValue());
    delay(30);
    sendCommand("setfilter yaw q " + yawFilterQ.getValue());
  } else if (name.equals("GraphToggles")) {
    CheckBox toggles = cp5.get(CheckBox.class, "GraphToggles");
    if (toggles != null) {
      for (int i = 0; i < graphEnabled.length; i++) {
        graphEnabled[i] = toggles.getState(i);
      }
    }
  }
}


// Button event handlers
void Send() {
  sendPIDValues();
}

void Save() {
  savePIDValues();
}

void Status() {
  requestStatus();
}

void Connect() {
  reconnect();
}

void Reset() {
  resetDrone();
}

void SendCommand() {
  String command = cp5.get(Textfield.class, "CommandLine").getText();
  if (!command.isEmpty()) {
    sendCommand(command);
    cp5.get(Textfield.class, "CommandLine").setText("");
  }
}

void UpdateGraph() {
  updateGraphParameters();
}

// Communication functions
void sendPIDValues() {
  try {
    kp_pitch = float(cp5.get(Textfield.class, "Kp_pitch").getText());
    ki_pitch = float(cp5.get(Textfield.class, "Ki_pitch").getText());
    kd_pitch = float(cp5.get(Textfield.class, "Kd_pitch").getText());
    
    kp_roll = float(cp5.get(Textfield.class, "Kp_roll").getText());
    ki_roll = float(cp5.get(Textfield.class, "Ki_roll").getText());
    kd_roll = float(cp5.get(Textfield.class, "Kd_roll").getText());
    
    kp_yaw = float(cp5.get(Textfield.class, "Kp_yaw").getText());
    ki_yaw = float(cp5.get(Textfield.class, "Ki_yaw").getText());
    kd_yaw = float(cp5.get(Textfield.class, "Kd_yaw").getText());
    
    sendCommand("set pitch kp " + kp_pitch+"\n");
    sendCommand("set pitch ki " + ki_pitch+"\n");
    delay(100);
    sendCommand("set pitch kd " + kd_pitch+"\n");
    sendCommand("set roll kp " + kp_roll+"\n");
    delay(100);
    sendCommand("set roll ki " + ki_roll+"\n");
    sendCommand("set roll kd " + kd_roll+"\n");
    delay(100);
    sendCommand("set yaw kp " + kp_yaw+"\n");
    sendCommand("set yaw ki " + ki_yaw+"\n");
    delay(50);
    sendCommand("set yaw kd " + kd_yaw+"\n");    
    
    addSerialMessage("Sent PID values");
  } catch (NumberFormatException e) {
    addSerialMessage("Invalid PID values");
  }
}

void sendCommand(String command) {
    println("[SEND] " + command);
  if (!isConnected) {
    addSerialMessage("Not connected");
    return;
  }
  
  try {
    if (myPort != null) {
      myPort.write(command + "\n");
    } else if (out != null) {
      out.println(command);
    }
    addSerialMessage("Sent: " + command);
  } catch (Exception e) {
    addSerialMessage("Send error: " + e.getMessage());
  }
}

void savePIDValues() {
  String[] lines = {
    "kp_pitch=" + kp_pitch,
    "ki_pitch=" + ki_pitch,
    "kd_pitch=" + kd_pitch,
    "kp_roll=" + kp_roll,
    "ki_roll=" + ki_roll,
    "kd_roll=" + kd_roll,
    "kp_yaw=" + kp_yaw,
    "ki_yaw=" + ki_yaw,
    "kd_yaw=" + kd_yaw
  };
  sendCommand("save");
  saveStrings("pid_values.txt", lines);
  addSerialMessage("PID values saved");
}

void requestStatus() {
  sendCommand("status");
}

void reconnect() {
  disconnect();
  delay(1000);
  setupCommunication();
}

void resetDrone() {
  sendCommand("reset");
}

void disconnect() {
  isConnected = false;
  keepReading = false;
  
  if (myPort != null) {
    myPort.stop();
    myPort = null;
  }
  
  if (socket != null) {
    try {
      socket.close();
    } catch (Exception e) {
      // Ignore
    }
    socket = null;
  }
  
  if (out != null) {
    out.close();
    out = null;
  }
  
  if (in != null) {
    try {
      in.close();
    } catch (Exception e) {
      // Ignore
    }
    in = null;
  }
  
  connectionStatus = "Disconnected";
  addSerialMessage("Disconnected");
}

void updateGraphParameters() {
  try {
    float newYMin = float(cp5.get(Textfield.class, "GraphYMin").getText());
    float newYMax = float(cp5.get(Textfield.class, "GraphYMax").getText());
    int newLength = int(cp5.get(Textfield.class, "GraphLength").getText());
    int newUpdateRate = int(cp5.get(Textfield.class, "GraphUpdateRate").getText());
    
    if (newYMin < newYMax && newLength > 0 && newUpdateRate > 0) {
      graphYMin = newYMin;
      graphYMax = newYMax;
      
      if (newLength != graphLength) {
        graphLength = newLength;
        // Reinitialize arrays with new length
        pitchGraph = new float[graphLength];
        rollGraph = new float[graphLength];
        yawGraph = new float[graphLength];
        pitchCorrGraph = new float[graphLength];
        rollCorrGraph = new float[graphLength];
        yawCorrGraph = new float[graphLength];
        initializeGraphs();
      }
      
      graphUpdateRate = newUpdateRate;
      addSerialMessage("Graph parameters updated");
    } else {
      addSerialMessage("Invalid graph parameters");
    }
  } catch (NumberFormatException e) {
    addSerialMessage("Invalid graph parameters");
  }
}

// FFT Functions
void updateFFT() {
  float value = 0;
  switch(selectedFFTIndex) {
    case 0: value = pitch; break;
    case 1: value = roll; break;
    case 2: value = yaw; break;
    case 3: value = pitchCorr; break;
    case 4: value = rollCorr; break;
    case 5: value = yawCorr; break;
  }
  
  // Add to FFT input buffer
  fftInput[fftIndex] = value;
  fftIndex = (fftIndex + 1) % fftInput.length;
}

void drawFFT(float x, float y, float w, float h) {
  // Select signal based on dropdown
  float[] signal;
  switch (selectedFFTIndex) {
    case 0: signal = pitchGraph; break;
    case 1: signal = rollGraph; break;
    case 2: signal = yawGraph; break;
    case 3: signal = pitchCorrGraph; break;
    case 4: signal = rollCorrGraph; break;
    case 5: signal = yawCorrGraph; break;
    default: return;
  }

  if (signal == null || signal.length == 0) return;

  // Prepare FFT input from signal
  for (int i = 0; i < fft.specSize(); i++) {
    fftInput[i] = (i < signal.length) ? signal[i] : 0;
  }
  fft.forward(fftInput);

  // Background and title
  noStroke();
  fill(20, 20, 20, 220);
  rect(x, y, w, h);

  fill(255);
  textSize(12);
  textAlign(LEFT, TOP);
  text("Frequency Domain - " + fftSources[selectedFFTIndex], x + 5, y + 5);

  // Setup for bars
  int bands = fft.specSize();
  float bandWidth = w / (float)bands;
  float graphBottom = y + h - 5;

  float maxVal = 1e-5;
  for (int i = 0; i < bands; i++) {
    float v = fft.getBand(i);
    if (v > maxVal) maxVal = v;
  }

  // Get hovered band from mouseX
  int hoveredBand = int((mouseX - x) / bandWidth);
  boolean inBounds = (mouseX >= x && mouseX < x + w && mouseY >= y && mouseY <= y + h);

  // Draw FFT bars and highlight hovered one
  for (int i = 0; i < bands; i++) {
    float v = fft.getBand(i);
    float normHeight = map(v, 0, maxVal, 0, h - 25);
    float bandX = x + i * bandWidth;

    if (inBounds && i == hoveredBand) {
      stroke(255, 255, 0); // Highlight
    } else {
      stroke(0, 255, 128);
    }

    line(bandX, graphBottom, bandX, graphBottom - normHeight);
  }

  // Tooltip with frequency & magnitude
  if (inBounds && hoveredBand >= 0 && hoveredBand < bands) {
    float freq = fft.indexToFreq(hoveredBand);
    float mag = fft.getBand(hoveredBand);

    String label = String.format("Freq: %.1f Hz\nMag: %.2f", freq, mag);

    float boxW = textWidth("Freq: 000.0 Hz") + 10;
    float boxH = 30;

    float tooltipX = constrain(mouseX + 10, 0, width - boxW - 1);
    float tooltipY = constrain(mouseY + 10, 0, height - boxH - 1);

    fill(0, 200);
    noStroke();
    rect(tooltipX, tooltipY, boxW, boxH, 5);

    fill(255);
    textSize(11);
    textAlign(LEFT, TOP);
    text("Freq: " + nf(freq, 0, 1) + " Hz", tooltipX + 5, tooltipY + 4);
    text("Mag: " + nf(mag, 0, 2), tooltipX + 5, tooltipY + 16);
  }
}


// Utility functions for 3D shapes
void cylinder(float radius, float height) {
  int sides = 12;
  float angle = 360.0 / sides;
  
  // Draw cylinder
  beginShape(QUAD_STRIP);
  for (int i = 0; i <= sides; i++) {
    float x = cos(radians(i * angle)) * radius;
    float z = sin(radians(i * angle)) * radius;
    vertex(x, -height/2, z);
    vertex(x, height/2, z);
  }
  endShape();
}

void cone(float radius, float height) {
  int sides = 8;
  float angle = 360.0 / sides;
  
  // Draw cone
  beginShape(TRIANGLES);
  for (int i = 0; i < sides; i++) {
    float x1 = cos(radians(i * angle)) * radius;
    float z1 = sin(radians(i * angle)) * radius;
    float x2 = cos(radians((i + 1) * angle)) * radius;
    float z2 = sin(radians((i + 1) * angle)) * radius;
    
    vertex(0, -height, 0);
    vertex(x1, 0, z1);
    vertex(x2, 0, z2);
  }
  endShape();
}


// Keyboard shortcuts
void keyPressed() {
  switch(key) {
    case 's':
    case 'S':
      sendPIDValues();
      break;
    case 'c':
    case 'C':
      reconnect();
      break;
    case 'r':
    case 'R':
      resetDrone();
      break;
    case ' ':
      requestStatus();
      break;
  }
}

// Cleanup on exit
void exit() {
  disconnect();
  if (minim != null) {
    minim.stop();
  }
  super.exit();
}
