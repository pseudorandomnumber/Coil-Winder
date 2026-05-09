#include <AccelStepper.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include "secrets.h"

// Forward declarations
void handleRoot();
void handleSet();
void handleStart();
void handleStop();
void handleHome();
void handleStatus();
void handleNotFound();
void readSerial();
void startMotion();
void stopMotion();
void processCommand(const String &raw);
long parseValue(const String &text);
float parseFloatValue(const String &text);
void updateTraverseLimits();
void handleHoming();
void handleRunning();

// I2S Shift Register pins for MKS DLC32 V2.1
const int SR_BCK_PIN = 16;
const int SR_DATA_PIN = 21;
const int SR_WS_PIN = 17;

// Traverse endstop input (Y-limit switch on ESP32)
const int TRAVERSE_HOME_PIN = 35;

uint8_t sr_state = 0; // Bit 0 (EN) is 0 -> enabled

static inline void sr_delay() {
  for(int i=0; i<4; i++) asm volatile("nop");
}

void updateShiftRegister() {
  GPIO.out_w1tc = (1 << SR_WS_PIN); // Latch LOW
  sr_delay();
  for (int i = 7; i >= 0; i--) {
    if ((sr_state >> i) & 1) {
      GPIO.out_w1ts = (1 << SR_DATA_PIN); // Data HIGH
    } else {
      GPIO.out_w1tc = (1 << SR_DATA_PIN); // Data LOW
    }
    sr_delay();
    GPIO.out_w1ts = (1 << SR_BCK_PIN); // Clock HIGH
    sr_delay();
    GPIO.out_w1tc = (1 << SR_BCK_PIN); // Clock LOW
    sr_delay();
  }
  GPIO.out_w1ts = (1 << SR_WS_PIN); // Latch HIGH
  sr_delay();
}

class ShiftStepper : public AccelStepper {
public:
  uint8_t step_bit;
  uint8_t dir_bit;
  bool dir_inverted;

  ShiftStepper(uint8_t step_b, uint8_t dir_b) : AccelStepper(DRIVER, 0, 0) {
    step_bit = step_b;
    dir_bit = dir_b;
    dir_inverted = false;
  }

  void setDirInvert(bool inv) {
    dir_inverted = inv;
  }

  virtual void setOutputPins(uint8_t mask) override {
    bool step_high = mask & 0b01;
    bool dir_high = mask & 0b10;

    if (dir_inverted) {
      dir_high = !dir_high;
    }

    if (step_high) sr_state |= (1 << step_bit);
    else sr_state &= ~(1 << step_bit);

    if (dir_high) sr_state |= (1 << dir_bit);
    else sr_state &= ~(1 << dir_bit);

    updateShiftRegister();
  }
};

ShiftStepper traverseLeft(1, 2);   // X_STEP (1), X_DIR (2)
ShiftStepper traverseRight(5, 6);  // Y_STEP (5), Y_DIR (6)
ShiftStepper spindle(3, 4);        // Z_STEP (3), Z_DIR (4)

long configuredTurns = 100;
float configuredStartMm = 0.0;
float configuredEndMm = 50.0;
float stepsPerMm = 10.0;
long startSteps = 0;
long endSteps = 500;
float traverseSpeed = 600.0;
float spindleSpeed = 1000.0;  // Spindle speed in steps/second

bool running = false;
bool homed = false;
bool homing = false;

const char* AP_SSID = "CoilWinder";
const char* AP_PASS = "coil1234";
// WIFI_SSID and WIFI_PASS are defined in secrets.h
IPAddress apIP(192, 168, 4, 1);
WebServer server(80);
String commandLine = "";

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Coil Winder</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 16px; color: #222; }
    .card { background: #f4f4f4; border-radius: 12px; padding: 16px; margin-bottom: 16px; }
    label { display: block; margin: 8px 0 4px; }
    input { width: 100%; padding: 10px; font-size: 18px; border: 1px solid #ccc; border-radius: 8px; }
    button { width: 48%; padding: 12px; font-size: 18px; margin-top: 12px; border: none; border-radius: 8px; }
    button.start { background: #28a745; color: white; }
    button.stop { background: #dc3545; color: white; }
    button.save { background: #007bff; color: white; width: 100%; }
    button.home { background: #f0ad4e; color: white; width: 100%; }
    .status { font-size: 18px; margin-top: 8px; }
  </style>
</head>
<body>
  <div class="card">
    <h1>Coil Winder</h1>
    <div class="status" id="status">Loading...</div>
  </div>
  <div class="card">
    <label for="turns">Turns (target)</label>
    <input type="number" id="turns" min="1" step="1" value="100" />
    <label for="start">Traverse start (mm)</label>
    <input type="number" id="start" min="0" step="0.1" value="0" />
    <label for="end">Traverse end (mm)</label>
    <input type="number" id="end" min="0" step="0.1" value="50" />
    <label for="stepsPerMm">Steps per mm</label>
    <input type="number" id="stepsPerMm" min="0.1" step="0.1" value="10" />
    <label for="speed">Traverse speed (steps/s)</label>
    <input type="number" id="speed" min="50" step="5" value="600" />
    <label for="spindleSpeed">Spindle speed (steps/s)</label>
    <input type="number" id="spindleSpeed" min="10" step="10" value="1000" />
    <button class="save" onclick="saveParams()">Save Parameters</button>
  </div>
  <div class="card">
    <button class="home" onclick="sendAction('home')">Home Traverse</button>
    <button class="start" onclick="sendAction('start')">START</button>
    <button class="stop" onclick="sendAction('stop')">STOP</button>
  </div>
  <script>
    async function loadStatus() {
      const response = await fetch('/status');
      const data = await response.json();
      document.getElementById('status').innerText =
        `TURNS=${data.turns} START=${data.startMm} END=${data.endMm} POS=${data.currentMm}mm HOMED=${data.homed ? 'YES' : 'NO'} STATE=${data.running ? 'RUNNING' : 'STOPPED'}`;
      document.getElementById('turns').value = data.turns;
      document.getElementById('start').value = data.startMm;
      document.getElementById('end').value = data.endMm;
      document.getElementById('stepsPerMm').value = data.stepsPerMm;
      document.getElementById('speed').value = data.speed;
      document.getElementById('spindleSpeed').value = data.spindleSpeed;
    }
    async function sendAction(action) {
      await fetch('/' + action);
      await loadStatus();
    }
    async function saveParams() {
      const turns = document.getElementById('turns').value;
      const start = document.getElementById('start').value;
      const end = document.getElementById('end').value;
      const stepsPerMm = document.getElementById('stepsPerMm').value;
      const speed = document.getElementById('speed').value;
      const spindleSpeed = document.getElementById('spindleSpeed').value;
      await fetch(`/set?turns=${turns}&start=${start}&end=${end}&stepsPerMm=${stepsPerMm}&speed=${speed}&spindleSpeed=${spindleSpeed}`);
      await loadStatus();
    }
    loadStatus();
    setInterval(loadStatus, 3000);
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(SR_BCK_PIN, OUTPUT);
  pinMode(SR_DATA_PIN, OUTPUT);
  pinMode(SR_WS_PIN, OUTPUT);
  digitalWrite(SR_WS_PIN, HIGH);
  updateShiftRegister();

  pinMode(TRAVERSE_HOME_PIN, INPUT_PULLUP);

  traverseLeft.setMaxSpeed(traverseSpeed);
  traverseLeft.setAcceleration(1000);
  traverseRight.setMaxSpeed(traverseSpeed);
  traverseRight.setAcceleration(1000);
  spindle.setMaxSpeed(spindleSpeed);
  spindle.setAcceleration(500);

  updateTraverseLimits();

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to WiFi %s", WIFI_SSID);
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(500);
    Serial.print('.');
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.printf("Connected to %s, IP=%s\n", WIFI_SSID, WiFi.localIP().toString().c_str());
    if (MDNS.begin("coilwinder")) {
      Serial.println("mDNS responder started: coilwinder.local");
    } else {
      Serial.println("mDNS setup failed");
    }
  } else {
    Serial.println();
    Serial.println("WiFi connection failed, still running AP mode.");
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/set", HTTP_GET, handleSet);
  server.on("/start", HTTP_GET, handleStart);
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/home", HTTP_GET, handleHome);
  server.on("/status", HTTP_GET, handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.printf("SoftAP started: %s\n", AP_SSID);
  Serial.printf("AP URL: http://%s\n", apIP.toString().c_str());
  Serial.println("Use http://coilwinder.local if your network supports mDNS.");
}

void loop() {
  server.handleClient();

  if (homing) {
    handleHoming();
  } else if (running) {
    handleRunning();
  }

  // Always run the spindle if it has a target speed
  spindle.run();

  readSerial();
}

void updateTraverseLimits() {
  startSteps = (long)(configuredStartMm * stepsPerMm + 0.5);
  endSteps = (long)(configuredEndMm * stepsPerMm + 0.5);
  if (endSteps < startSteps) {
    long swap = startSteps;
    startSteps = endSteps;
    endSteps = swap;
  }
}

void setTraversePosition(long position) {
  traverseLeft.setCurrentPosition(position);
  traverseRight.setCurrentPosition(position);
}

void moveTraverseTo(long target) {
  traverseLeft.moveTo(target);
  traverseRight.moveTo(target);
}

void handleRunning() {
  if (traverseLeft.distanceToGo() == 0 && traverseRight.distanceToGo() == 0) {
    long nextTarget = (traverseLeft.currentPosition() >= endSteps) ? startSteps : endSteps;
    moveTraverseTo(nextTarget);
  }
  traverseLeft.run();
  traverseRight.run();
}

void handleHoming() {
  static unsigned long lastDebugTime = 0;
  
  if (digitalRead(TRAVERSE_HOME_PIN) == HIGH) {
    // Endstop triggered, both motors have reached home
    traverseLeft.stop();
    traverseRight.stop();
    setTraversePosition(startSteps);
    homing = false;
    homed = true;
    moveTraverseTo(endSteps);
    Serial.println("Traverse homing complete.");
  } else {
    // Continue moving both motors towards endstop
    traverseLeft.run();
    traverseRight.run();
    
    // Debug output every 500ms
    if (millis() - lastDebugTime > 500) {
      Serial.printf("Homing: endstop_pin=%d, left_pos=%ld, right_pos=%ld\n", 
                    digitalRead(TRAVERSE_HOME_PIN), 
                    traverseLeft.currentPosition(), 
                    traverseRight.currentPosition());
      lastDebugTime = millis();
    }
  }
}

void handleRoot() {
  server.send_P(200, "text/html", index_html);
}

void handleSet() {
  if (server.hasArg("turns")) {
    long value = server.arg("turns").toInt();
    if (value > 0) configuredTurns = value;
  }
  if (server.hasArg("start")) {
    float value = server.arg("start").toFloat();
    if (value >= 0) configuredStartMm = value;
  }
  if (server.hasArg("end")) {
    float value = server.arg("end").toFloat();
    if (value >= 0) configuredEndMm = value;
  }
  if (server.hasArg("stepsPerMm")) {
    float value = server.arg("stepsPerMm").toFloat();
    if (value > 0) stepsPerMm = value;
  }
  if (server.hasArg("speed")) {
    float value = server.arg("speed").toFloat();
    if (value > 0.0f) {
      traverseSpeed = value;
      traverseLeft.setMaxSpeed(traverseSpeed);
      traverseRight.setMaxSpeed(traverseSpeed);
    }
  }
  if (server.hasArg("spindleSpeed")) {
    float value = server.arg("spindleSpeed").toFloat();
    if (value > 0.0f) {
      spindleSpeed = value;
      spindle.setMaxSpeed(spindleSpeed);
    }
  }

  updateTraverseLimits();
  server.send(200, "text/plain", "OK");
}

void handleStart() {
  if (!homed) {
    Serial.println("Cannot start: traverse not homed.");
  } else if (!running) {
    startMotion();
  }
  server.send(200, "text/plain", homed ? "OK" : "NOT_HOMED");
}

void handleStop() {
  stopMotion();
  server.send(200, "text/plain", "OK");
}

void handleHome() {
  if (!homing) {
    homing = true;
    traverseLeft.setMaxSpeed(traverseSpeed * 0.25);
    traverseRight.setMaxSpeed(traverseSpeed * 0.25);
    traverseLeft.moveTo(-100000);
    traverseRight.moveTo(-100000);
    Serial.println("Starting traverse homing...");
  }
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  float currentMm = (float)traverseLeft.currentPosition() / stepsPerMm;
  String json = "{";
  json += "\"turns\":" + String(configuredTurns) + ",";
  json += "\"startMm\":" + String(configuredStartMm, 1) + ",";
  json += "\"endMm\":" + String(configuredEndMm, 1) + ",";
  json += "\"stepsPerMm\":" + String(stepsPerMm, 1) + ",";
  json += "\"speed\":" + String(traverseSpeed, 1) + ",";
  json += "\"spindleSpeed\":" + String(spindleSpeed, 1) + ",";
  json += "\"currentMm\":" + String(currentMm, 1) + ",";
  json += "\"homed\":" + String(homed ? "true" : "false") + ",";
  json += "\"running\":" + String(running ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (commandLine.length() > 0) {
        processCommand(commandLine);
        commandLine = "";
      }
    } else {
      commandLine += c;
    }
  }
}

void processCommand(const String &raw) {
  String cmd = raw;
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.startsWith("TURNS")) {
    long value = parseValue(cmd.substring(5));
    if (value > 0) configuredTurns = value;
    return;
  }
  if (cmd.startsWith("STARTPOS")) {
    float value = parseFloatValue(cmd.substring(8));
    if (value >= 0) configuredStartMm = value;
    updateTraverseLimits();
    return;
  }
  if (cmd.startsWith("ENDPOS")) {
    float value = parseFloatValue(cmd.substring(6));
    if (value >= 0) configuredEndMm = value;
    updateTraverseLimits();
    return;
  }
  if (cmd.startsWith("STEPSMM")) {
    float value = parseFloatValue(cmd.substring(7));
    if (value > 0) stepsPerMm = value;
    updateTraverseLimits();
    return;
  }
  if (cmd.startsWith("SPEED")) {
    float value = parseFloatValue(cmd.substring(5));
    if (value > 0.0f) {
      traverseSpeed = value;
      traverseLeft.setMaxSpeed(traverseSpeed);
      traverseRight.setMaxSpeed(traverseSpeed);
    }
    return;
  }
  if (cmd == "HOME") {
    handleHome();
    return;
  }
  if (cmd == "START") {
    startMotion();
    return;
  }
  if (cmd == "STOP") {
    stopMotion();
    return;
  }
}

long parseValue(const String &text) {
  String trimmed = text;
  trimmed.trim();
  return trimmed.toInt();
}

float parseFloatValue(const String &text) {
  String trimmed = text;
  trimmed.trim();
  return trimmed.toFloat();
}

void startMotion() {
  if (!homed) {
    Serial.println("Cannot start: traverse not homed.");
    return;
  }

  setTraversePosition(startSteps);
  moveTraverseTo(endSteps);
  traverseLeft.setMaxSpeed(traverseSpeed);
  traverseRight.setMaxSpeed(traverseSpeed);
  spindle.setMaxSpeed(spindleSpeed);
  spindle.moveTo(2147483647);  // Move to a very large number to spin continuously
  running = true;
  Serial.println("Winding started.");
}

void stopMotion() {
  running = false;
  traverseLeft.stop();
  traverseRight.stop();
  spindle.stop();
  Serial.println("Winding stopped.");
}
