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
void handleJog();
void handleSetStartPos();
void handleSetEndPos();
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
void handleJogging();

// I2S Shift Register pins for MKS DLC32 V2.1
const int SR_BCK_PIN = 16;
const int SR_DATA_PIN = 21;
const int SR_WS_PIN = 17;

// Limit Switches
const int X_LIMIT_PIN = 36; // Traverse Left
const int Y_LIMIT_PIN = 35; // Traverse Right
const int PROBE_PIN = 22;   // E-Stop / General Probe

uint8_t sr_state = 0; // Bit 0 (EN) is 0 -> enabled

void IRAM_ATTR updateShiftRegister() {
  GPIO.out_w1tc = (1 << SR_WS_PIN); // Latch LOW
  asm volatile("nop; nop; nop; nop;");
  for (int i = 7; i >= 0; i--) {
    if ((sr_state >> i) & 1) {
      GPIO.out_w1ts = (1 << SR_DATA_PIN); // Data HIGH
    } else {
      GPIO.out_w1tc = (1 << SR_DATA_PIN); // Data LOW
    }
    asm volatile("nop; nop; nop; nop;");
    GPIO.out_w1ts = (1 << SR_BCK_PIN); // Clock HIGH
    asm volatile("nop; nop; nop; nop;");
    GPIO.out_w1tc = (1 << SR_BCK_PIN); // Clock LOW
  }
  asm volatile("nop; nop; nop; nop;");
  GPIO.out_w1ts = (1 << SR_WS_PIN); // Latch HIGH
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

  virtual void step1(long step) override {
    bool dir_high = (_direction == DIRECTION_CW);
    if (dir_inverted) dir_high = !dir_high;

    if (dir_high) sr_state |= (1 << dir_bit);
    else sr_state &= ~(1 << dir_bit);

    // Step HIGH
    sr_state |= (1 << step_bit);
    updateShiftRegister();

    delayMicroseconds(1);

    // Step LOW
    sr_state &= ~(1 << step_bit);
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

// Default speeds
volatile float traverseSpeed = 4000.0;
volatile float spindleSpeed = 8000.0;  // Spindle speed in steps/second

volatile bool running = false;
volatile bool jogging = false;
volatile bool homed = false;
volatile bool homing = false;
volatile int homingState = 0;

// Command flags: set by web/serial handlers (Core 0), consumed by loop() (Core 1)
// This prevents unsafe cross-core AccelStepper access.
volatile bool cmd_home = false;
volatile bool cmd_start = false;
volatile bool cmd_stop = false;
volatile bool cmd_jog = false;
volatile float cmd_jog_dist = 0.0f;

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
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 16px; background: #eef2f5; color: #333; }
    .header { text-align: center; margin-bottom: 24px; }
    .header h1 { margin: 12px 0 4px; color: #2c3e50; font-size: 28px; }
    .header p { margin: 0; font-style: italic; color: #7f8c8d; font-size: 16px; }
    .card { background: #fff; border-radius: 12px; padding: 20px; margin-bottom: 20px; box-shadow: 0 2px 10px rgba(0,0,0,0.05); }
    h2 { margin-top: 0; color: #2c3e50; font-size: 20px; border-bottom: 2px solid #ecf0f1; padding-bottom: 8px; }
    .stats-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 16px; }
    .stat-box { background: #f8f9fa; padding: 12px; border-radius: 8px; text-align: center; border: 1px solid #e9ecef; }
    .stat-box .label { font-size: 12px; color: #6c757d; text-transform: uppercase; letter-spacing: 0.5px; }
    .stat-box .value { font-size: 20px; font-weight: bold; color: #2c3e50; margin-top: 4px; }
    label { display: block; margin: 12px 0 4px; font-weight: 600; font-size: 14px; color: #495057; }
    input { width: 100%; box-sizing: border-box; padding: 12px; font-size: 16px; border: 1px solid #ced4da; border-radius: 8px; transition: border-color 0.15s; }
    input:focus { border-color: #80bdff; outline: 0; }
    .btn-group { display: flex; gap: 10px; margin-top: 16px; flex-wrap: wrap; }
    button { flex: 1; padding: 14px; font-size: 16px; font-weight: bold; border: none; border-radius: 8px; cursor: pointer; transition: opacity 0.2s; text-transform: uppercase; }
    button:active { opacity: 0.8; }
    button.start { background: #28a745; color: white; }
    button.stop { background: #dc3545; color: white; min-width: 100%; margin-top: 10px; font-size: 20px; }
    button.save { background: #007bff; color: white; width: 100%; margin-top: 16px; }
    button.home { background: #f0ad4e; color: white; }
    button.speed { background: #17a2b8; color: white; min-width: 20%; padding: 10px; font-size: 14px; }
    button.jog { background: #6c757d; color: white; padding: 10px; font-size: 14px; }
    button.set-pos { background: #ffc107; color: #212529; padding: 10px; font-size: 14px; }
    .speed-control, .jog-control { display: flex; gap: 8px; margin-top: 8px; flex-wrap: wrap; }
    .badge { display: inline-block; padding: 4px 8px; border-radius: 4px; font-size: 12px; font-weight: bold; color: white; }
    .badge.green { background: #28a745; }
    .badge.red { background: #dc3545; }
    .badge.gray { background: #6c757d; }
  </style>
</head>
<body>
  <div class="header">
    <h1>Coil Winder</h1>
    <p>WAAS - Winding as a service</p>
  </div>
  
  <div class="card">
    <h2>Real-time Stats</h2>
    <div class="stats-grid">
      <div class="stat-box"><div class="label">Position (mm)</div><div class="value" id="stat-pos">0.0</div></div>
      <div class="stat-box"><div class="label">State</div><div class="value" id="stat-state">STOPPED</div></div>
      <div class="stat-box"><div class="label">Homed</div><div class="value" id="stat-homed">NO</div></div>
      <div class="stat-box"><div class="label">Speed (steps/s)</div><div class="value" id="stat-speed">0</div></div>
      
      <div class="stat-box"><div class="label">X-Limit (Left)</div><div class="value" id="stat-xlim"><span class="badge gray">Unknown</span></div></div>
      <div class="stat-box"><div class="label">Y-Limit (Right)</div><div class="value" id="stat-ylim"><span class="badge gray">Unknown</span></div></div>
      <div class="stat-box" style="grid-column: span 2;"><div class="label">Probe (E-STOP)</div><div class="value" id="stat-probe"><span class="badge gray">Unknown</span></div></div>
    </div>
  </div>

  <div class="card">
    <h2>Control</h2>
    <div class="btn-group">
      <button class="home" onclick="sendAction('home')">Home Traverse</button>
      <button class="start" onclick="sendAction('start')">Start Winding</button>
    </div>
    <button class="stop" onclick="sendAction('stop')">E-STOP / HALT</button>
    
    <div style="margin-top: 20px; border-top: 1px solid #ecf0f1; padding-top: 16px;">
      <label>Jog Traverse Axis</label>
      <div class="jog-control">
        <button class="jog" onclick="jog(-10)">-10mm</button>
        <button class="jog" onclick="jog(-1)">-1mm</button>
        <button class="jog" onclick="jog(1)">+1mm</button>
        <button class="jog" onclick="jog(10)">+10mm</button>
      </div>
    </div>

    <div style="margin-top: 20px; border-top: 1px solid #ecf0f1; padding-top: 16px;">
      <label>Set Positions</label>
      <div class="jog-control">
        <button class="set-pos" onclick="setStartPos()">Set START to Current</button>
        <button class="set-pos" onclick="setEndPos()">Set END to Current</button>
      </div>
    </div>
    
    <div style="margin-top: 20px; border-top: 1px solid #ecf0f1; padding-top: 16px;">
      <label>Live Speed Override</label>
      <div class="speed-control">
        <button class="speed" onclick="adjustSpeed(0.5)">50%</button>
        <button class="speed" onclick="adjustSpeed(1.0)">100%</button>
        <button class="speed" onclick="adjustSpeed(1.5)">150%</button>
        <button class="speed" onclick="adjustSpeed(2.0)">200%</button>
      </div>
    </div>
  </div>

  <div class="card">
    <h2>Configuration</h2>
    <label for="turns">Turns (target)</label>
    <input type="number" id="turns" min="1" step="1" />
    <label for="start">Traverse start (mm)</label>
    <input type="number" id="start" min="0" step="0.1" />
    <label for="end">Traverse end (mm)</label>
    <input type="number" id="end" min="0" step="0.1" />
    <label for="stepsPerMm">Steps per mm</label>
    <input type="number" id="stepsPerMm" min="0.1" step="0.1" />
    <label for="speed">Base Traverse speed (steps/s)</label>
    <input type="number" id="speed" min="50" step="5" />
    <label for="spindleSpeed">Base Spindle speed (steps/s)</label>
    <input type="number" id="spindleSpeed" min="10" step="10" />
    <button class="save" onclick="saveParams()">Save Parameters</button>
  </div>

  <script>
    let firstLoad = true;
    let baseSpindleSpeed = 8000;
    let baseTraverseSpeed = 4000;

    function updateBadge(id, triggered) {
      const el = document.getElementById(id);
      if(!el) return;
      if(triggered) {
        el.innerHTML = '<span class="badge red">TRIGGERED</span>';
      } else {
        el.innerHTML = '<span class="badge green">OPEN</span>';
      }
    }

    async function loadStatus() {
      try {
        const response = await fetch('/status');
        const data = await response.json();
        
        const posEl = document.getElementById('stat-pos');
        if(posEl) posEl.innerText = (data.currentMm !== undefined && data.currentMm !== null) ? data.currentMm.toFixed(1) : "0.0";
        
        const stateEl = document.getElementById('stat-state');
        if(stateEl) stateEl.innerText = data.running ? 'RUNNING' : (data.homing ? 'HOMING' : (data.jogging ? 'JOGGING' : 'STOPPED'));
        
        const homedEl = document.getElementById('stat-homed');
        if(homedEl) homedEl.innerText = data.homed ? 'YES' : 'NO';
        
        const speedEl = document.getElementById('stat-speed');
        if(speedEl) speedEl.innerText = (data.speed !== undefined && data.speed !== null) ? data.speed.toFixed(0) : "0";
        
        updateBadge('stat-xlim', data.xlim);
        updateBadge('stat-ylim', data.ylim);
        updateBadge('stat-probe', data.probe);

        if (firstLoad) {
          document.getElementById('turns').value = data.turns;
          document.getElementById('start').value = data.startMm;
          document.getElementById('end').value = data.endMm;
          document.getElementById('stepsPerMm').value = data.stepsPerMm;
          document.getElementById('speed').value = data.speed;
          document.getElementById('spindleSpeed').value = data.spindleSpeed;
          baseTraverseSpeed = data.speed;
          baseSpindleSpeed = data.spindleSpeed;
          firstLoad = false;
        }
      } catch (e) {
        console.error("Error loading status:", e);
      }
    }
    
    async function sendAction(action) {
      const response = await fetch('/' + action);
      const text = await response.text();
      if (text === 'NOT_HOMED') {
        alert('Cannot start: Traverse axis not homed yet!');
      }
      await loadStatus();
    }
    
    async function jog(dist) {
      await fetch('/jog?dist=' + dist);
      await loadStatus();
    }

    async function setStartPos() {
      await fetch('/set_start_pos');
      firstLoad = true; // Force reload of values
      await loadStatus();
    }

    async function setEndPos() {
      await fetch('/set_end_pos');
      firstLoad = true;
      await loadStatus();
    }

    async function saveParams() {
      const turns = document.getElementById('turns').value;
      const start = document.getElementById('start').value;
      const end = document.getElementById('end').value;
      const stepsPerMm = document.getElementById('stepsPerMm').value;
      const speed = document.getElementById('speed').value;
      const spindleSpeed = document.getElementById('spindleSpeed').value;
      
      baseTraverseSpeed = speed;
      baseSpindleSpeed = spindleSpeed;
      
      await fetch(`/set?turns=${turns}&start=${start}&end=${end}&stepsPerMm=${stepsPerMm}&speed=${speed}&spindleSpeed=${spindleSpeed}`);
      await loadStatus();
    }

    async function adjustSpeed(multiplier) {
      const newSpeed = baseTraverseSpeed * multiplier;
      const newSpindle = baseSpindleSpeed * multiplier;
      await fetch(`/set?speed=${newSpeed}&spindleSpeed=${newSpindle}`);
      await loadStatus();
    }

    loadStatus();
    setInterval(loadStatus, 1000);
  </script>
</body>
</html>
)rawliteral";

void webServerTask(void * pvParameters) {
  for (;;) {
    server.handleClient();
    readSerial();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(SR_BCK_PIN, OUTPUT);
  pinMode(SR_DATA_PIN, OUTPUT);
  pinMode(SR_WS_PIN, OUTPUT);
  digitalWrite(SR_WS_PIN, HIGH);
  updateShiftRegister();

  // GPIO 36 and 35 are input-only pins on the ESP32 and do NOT support INPUT_PULLUP.
  // The MKS DLC32 board has external 10k pull-ups on these limit switch inputs.
  pinMode(X_LIMIT_PIN, INPUT);
  pinMode(Y_LIMIT_PIN, INPUT);
  pinMode(PROBE_PIN, INPUT_PULLUP);

  traverseLeft.setMinPulseWidth(0);
  traverseLeft.setMaxSpeed(traverseSpeed);
  traverseLeft.setAcceleration(10000);
  
  traverseRight.setMinPulseWidth(0);
  traverseRight.setMaxSpeed(traverseSpeed);
  traverseRight.setAcceleration(10000);

  spindle.setMinPulseWidth(0);
  spindle.setMaxSpeed(spindleSpeed);
  spindle.setAcceleration(10000);

  updateTraverseLimits();

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP("CoilWinder", "coil1234");

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to WiFi %s", WIFI_SSID);
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(500);
    Serial.print('.');
  }

  if (WiFi.status() == WL_CONNECTED) {
    WiFi.mode(WIFI_STA); // Disable AP mode if connected to home network
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
  server.on("/jog", HTTP_GET, handleJog);
  server.on("/set_start_pos", HTTP_GET, handleSetStartPos);
  server.on("/set_end_pos", HTTP_GET, handleSetEndPos);
  server.onNotFound(handleNotFound);
  server.begin();

  xTaskCreatePinnedToCore(
    webServerTask,
    "WebServer",
    8192,
    NULL,
    1,
    NULL,
    0
  );

  Serial.println("System ready.");
}

void loop() {
  // --- Process commands queued by web handlers (Core 0) safely on Core 1 ---
  if (cmd_stop) {
    cmd_stop = false; cmd_home = false; cmd_start = false; cmd_jog = false;
    running = false; homing = false; jogging = false;
    traverseLeft.stop(); traverseRight.stop(); spindle.stop();
    Serial.println("Motion stopped.");
  }

  if (cmd_home && !homing && !running && !jogging) {
    cmd_home = false;
    homing = true; homingState = 0;
    traverseLeft.setMaxSpeed(traverseSpeed * 0.5);
    traverseRight.setMaxSpeed(traverseSpeed * 0.5);
    traverseLeft.moveTo(traverseLeft.currentPosition() - 100000);
    traverseRight.moveTo(traverseRight.currentPosition() - 100000);
    Serial.println("Homing: Seeking endstop...");
  }

  if (cmd_jog && !homing && !running) {
    cmd_jog = false;
    long steps = (long)(cmd_jog_dist * stepsPerMm);
    traverseLeft.setMaxSpeed(traverseSpeed);
    traverseRight.setMaxSpeed(traverseSpeed);
    traverseLeft.move(steps); traverseRight.move(steps);
    jogging = true;
  }

  if (cmd_start && !homing && !running && !jogging) {
    cmd_start = false;
    startMotion();
  }

  // EMERGENCY STOP PROBE CHECK (Active LOW)
  if (digitalRead(PROBE_PIN) == LOW && (running || homing || jogging)) {
    running = false; homing = false; jogging = false;
    traverseLeft.stop(); traverseRight.stop(); spindle.stop();
    Serial.println("E-STOP TRIGGERED! All motion halted.");
  }

  if (homing) {
    handleHoming();
  } else if (running) {
    handleRunning();
  } else if (jogging) {
    handleJogging();
  }

  spindle.run();
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

void handleJogging() {
  traverseLeft.run();
  traverseRight.run();
  if (traverseLeft.distanceToGo() == 0 && traverseRight.distanceToGo() == 0) {
    jogging = false;
  }
}

void handleHoming() {
  // Switches are normally-open, pull pin to GND when triggered -> LOW = triggered
  bool x_endstop = (digitalRead(X_LIMIT_PIN) == LOW);
  bool y_endstop = (digitalRead(Y_LIMIT_PIN) == LOW);
  
  if (homingState == 0) { // Seeking (Negative Direction)
    if (x_endstop) traverseLeft.stop();
    if (y_endstop) traverseRight.stop();
    
    if (traverseLeft.distanceToGo() == 0 && traverseRight.distanceToGo() == 0) {
      homingState = 4;
    }
  } else if (homingState == 4) { // wait stop
    if (traverseLeft.distanceToGo() == 0 && traverseRight.distanceToGo() == 0) {
      traverseLeft.setMaxSpeed(traverseSpeed * 0.25);
      traverseRight.setMaxSpeed(traverseSpeed * 0.25);
      traverseLeft.moveTo(traverseLeft.currentPosition() + 100000); // Pull away (Positive Direction)
      traverseRight.moveTo(traverseRight.currentPosition() + 100000);
      homingState = 1;
    }
  } else if (homingState == 1) { // Pulling away
    if (!x_endstop) traverseLeft.stop();
    if (!y_endstop) traverseRight.stop();

    if (traverseLeft.distanceToGo() == 0 && traverseRight.distanceToGo() == 0) {
      homingState = 5;
    }
  } else if (homingState == 5) { // wait stop
    if (traverseLeft.distanceToGo() == 0 && traverseRight.distanceToGo() == 0) {
      traverseLeft.setMaxSpeed(traverseSpeed * 0.05);
      traverseRight.setMaxSpeed(traverseSpeed * 0.05);
      traverseLeft.moveTo(traverseLeft.currentPosition() - 100000); // Slow seek (Negative Direction)
      traverseRight.moveTo(traverseRight.currentPosition() - 100000);
      homingState = 2;
    }
  } else if (homingState == 2) { // Slow seek
    if (x_endstop) traverseLeft.stop();
    if (y_endstop) traverseRight.stop();

    if (traverseLeft.distanceToGo() == 0 && traverseRight.distanceToGo() == 0) {
      homingState = 6;
    }
  } else if (homingState == 6) { // wait stop
    if (traverseLeft.distanceToGo() == 0 && traverseRight.distanceToGo() == 0) {
      traverseLeft.setMaxSpeed(traverseSpeed * 0.2);
      traverseRight.setMaxSpeed(traverseSpeed * 0.2);
      long pullOffSteps = (long)(2.0 * stepsPerMm + 0.5);
      traverseLeft.moveTo(traverseLeft.currentPosition() + pullOffSteps); // Final pull off (Positive Direction)
      traverseRight.moveTo(traverseRight.currentPosition() + pullOffSteps);
      homingState = 3;
    }
  } else if (homingState == 3) { // Pull off
    if (traverseLeft.distanceToGo() == 0 && traverseRight.distanceToGo() == 0) {
      setTraversePosition(startSteps);
      homing = false;
      homed = true;
      moveTraverseTo(endSteps);
      traverseLeft.setMaxSpeed(traverseSpeed);
      traverseRight.setMaxSpeed(traverseSpeed);
      Serial.println("Traverse homing complete.");
    }
  }

  if (homing) {
    traverseLeft.run();
    traverseRight.run();
  }
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
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

void handleJog() {
  if (running || homing) {
    server.send(400, "text/plain", "Busy");
    return;
  }
  if (server.hasArg("dist")) {
    cmd_jog_dist = server.arg("dist").toFloat();
    cmd_jog = true;
  }
  server.send(200, "text/plain", "OK");
}

void handleSetStartPos() {
  configuredStartMm = (float)traverseLeft.currentPosition() / stepsPerMm;
  updateTraverseLimits();
  server.send(200, "text/plain", "OK");
}

void handleSetEndPos() {
  configuredEndMm = (float)traverseLeft.currentPosition() / stepsPerMm;
  updateTraverseLimits();
  server.send(200, "text/plain", "OK");
}

void handleStart() {
  if (!homed) {
    server.send(200, "text/plain", "NOT_HOMED");
    return;
  }
  cmd_start = true;
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  cmd_stop = true;
  server.send(200, "text/plain", "OK");
}

void handleHome() {
  cmd_home = true;
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  float currentMm = (float)traverseLeft.currentPosition() / stepsPerMm;
  if (isnan(currentMm) || isinf(currentMm)) currentMm = 0.0;
  
  String json = "{";
  json += "\"turns\":" + String(configuredTurns) + ",";
  json += "\"startMm\":" + String(configuredStartMm, 1) + ",";
  json += "\"endMm\":" + String(configuredEndMm, 1) + ",";
  json += "\"stepsPerMm\":" + String(stepsPerMm, 1) + ",";
  json += "\"speed\":" + String(traverseSpeed, 1) + ",";
  json += "\"spindleSpeed\":" + String(spindleSpeed, 1) + ",";
  json += "\"currentMm\":" + String(currentMm, 1) + ",";
  json += "\"homed\":" + (homed ? String("true") : String("false")) + ",";
  json += "\"running\":" + (running ? String("true") : String("false")) + ",";
  json += "\"homing\":" + (homing ? String("true") : String("false")) + ",";
  json += "\"jogging\":" + (jogging ? String("true") : String("false")) + ",";
  json += "\"xlim\":" + (digitalRead(X_LIMIT_PIN) == LOW ? String("true") : String("false")) + ",";
  json += "\"ylim\":" + (digitalRead(Y_LIMIT_PIN) == LOW ? String("true") : String("false")) + ",";
  json += "\"probe\":" + (digitalRead(PROBE_PIN) == LOW ? String("true") : String("false"));
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
  jogging = false;
  traverseLeft.stop();
  traverseRight.stop();
  spindle.stop();
  Serial.println("Winding stopped.");
}
