// ============================================================================
// Predictive Automotive Climate Control System - V0.1 Team Starter
//
// Origin:
//   Extends the Car Cabin Temperature Prediction System V6.0.
//
// What this starter keeps:
//   - ESP32 + DS18B20 cabin-temperature sensing
//   - Newton-law thermal prediction concept
//   - Adaptive k estimation from measured temperature data
//   - ESP32 Wi-Fi access point + local web interface
//
// What this starter adds:
//   - User-selectable cabin target temperature
//   - Automatic HVAC decision logic
//   - Desired HVAC temperature/fan/A/C/recirculation state
//   - A clean actuator abstraction for later physical vehicle integration
//   - A simplified single live prediction model instead of 30/45/60 s test models
//
// IMPORTANT:
//   applyHVACCommand() is intentionally a BENCH-MODE stub. It only prints the
//   desired command over Serial. It does NOT connect to or control vehicle
//   electronics yet. Add the physical actuator interface only after the logic
//   works reliably on the bench.
// ============================================================================

#if !defined(ESP32)
#error "This project is intended for an ESP32 board."
#endif

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

// ============================================================================
// HARDWARE
// ============================================================================

#define ONE_WIRE_BUS 13

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ============================================================================
// WI-FI ACCESS POINT
// ============================================================================

const char* AP_SSID = "CarClimate";
const char* AP_PASSWORD = "CarClimate123";

AsyncWebServer server(80);

// ============================================================================
// SYSTEM SETTINGS
// ============================================================================

const unsigned long TEMPERATURE_INTERVAL_MS = 1000;
const unsigned long MODEL_UPDATE_INTERVAL_MS = 10000;
const unsigned long STATE_LOG_INTERVAL_MS = 5000;

// User-selectable cabin target range.
const float MIN_TARGET_TEMP_F = 60.0f;
const float MAX_TARGET_TEMP_F = 85.0f;
const float TARGET_TOLERANCE_F = 1.0f;

// Initial default target shown in the UI.
float targetTempF = 72.0f;

// Start disabled so physical actuation can never begin just because the ESP32
// powered on. The user explicitly enables Auto Climate from the web UI.
bool autoControlEnabled = false;

// ---------------------------------------------------------------------------
// MODEL REFERENCE TEMPERATURES
// ---------------------------------------------------------------------------
// Newton's Law requires an effective equilibrium/reference temperature.
// These are STARTING calibration values, not the user target.
//
// Cooling example:
//   Cabin = 100 F, target = 72 F, effective HVAC reference ~= 55 F.
//
// Heating example:
//   Cabin = 45 F, target = 72 F, effective HVAC reference ~= 95 F.
//
// Real-car testing should later calibrate these values (or replace this simple
// model with a reference that depends on fan speed / vent temperature).
const float MODEL_COOLING_REFERENCE_F = 55.0f;
const float MODEL_HEATING_REFERENCE_F = 95.0f;

// Ignore the earliest transient before estimating k.
const float MODEL_IGNORE_FIRST_SECONDS = 20.0f;

// Estimate k from at most the most recent 60 seconds of valid data.
const float MODEL_WINDOW_SECONDS = 60.0f;

// Limit and smooth k updates so sensor noise does not produce huge swings.
const float MAX_K_CHANGE_FRACTION = 0.15f;
const float K_SMOOTH_OLD_WEIGHT = 0.70f;
const float K_SMOOTH_NEW_WEIGHT = 0.30f;

// One sample per second gives 15 minutes of history.
#define MAX_SAMPLES 900

float temperatureSamples[MAX_SAMPLES];
unsigned long sampleTimes[MAX_SAMPLES];
int sampleCount = 0;

// ============================================================================
// TEMPERATURE + MODEL STATE
// ============================================================================

float currentTempF = NAN;
String temperatureText = "--";
String systemStatus = "Starting...";

float adaptiveK = -1.0f;
float predictedRemainingSeconds = -1.0f;
unsigned long modelSessionStartMillis = 0;

unsigned long lastTemperatureRead = 0;
unsigned long lastModelUpdate = 0;
unsigned long lastStateLog = 0;

// ============================================================================
// HVAC CONTROL TYPES
// ============================================================================

enum ThermalDirection
{
  DIRECTION_NONE,
  DIRECTION_COOLING,
  DIRECTION_HEATING
};

struct HVACCommand
{
  bool active;
  float setpointF;
  int fanLevel;              // 1-7 for this prototype; 0 means no command.
  bool acOn;
  bool recirculationOn;
  String mode;
};

HVACCommand desiredHVAC =
{
  false,
  72.0f,
  0,
  false,
  false,
  "AUTO OFF"
};

HVACCommand lastAppliedHVAC =
{
  false,
  -999.0f,
  -1,
  false,
  false,
  "UNINITIALIZED"
};

ThermalDirection modelDirection = DIRECTION_NONE;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

float clampFloat(float value, float low, float high)
{
  if(value < low) return low;
  if(value > high) return high;
  return value;
}

bool isSensorFailure(float temp)
{
  return isnan(temp) || temp <= -190.0f || temp >= 200.0f;
}

ThermalDirection getRequiredDirection(float current, float target)
{
  if(isSensorFailure(current))
  {
    return DIRECTION_NONE;
  }

  float error = current - target;

  if(fabs(error) <= TARGET_TOLERANCE_F)
  {
    return DIRECTION_NONE;
  }

  return (error > 0.0f)
      ? DIRECTION_COOLING
      : DIRECTION_HEATING;
}

float getModelReferenceTemp(ThermalDirection direction)
{
  if(direction == DIRECTION_COOLING)
  {
    return MODEL_COOLING_REFERENCE_F;
  }

  if(direction == DIRECTION_HEATING)
  {
    return MODEL_HEATING_REFERENCE_F;
  }

  return targetTempF;
}

String directionToString(ThermalDirection direction)
{
  if(direction == DIRECTION_COOLING) return "COOLING";
  if(direction == DIRECTION_HEATING) return "HEATING";
  return "NONE";
}

// ============================================================================
// TEMPERATURE SENSOR
// ============================================================================

void updateTemperatureStatus()
{
  if(isSensorFailure(currentTempF))
  {
    systemStatus = "Sensor Failure";
    return;
  }

  float error = currentTempF - targetTempF;

  if(fabs(error) <= TARGET_TOLERANCE_F)
  {
    systemStatus = "Target Reached";
  }
  else if(error > 0.0f)
  {
    systemStatus = "Cabin Above Target";
  }
  else
  {
    systemStatus = "Cabin Below Target";
  }
}

void readTemperature()
{
  sensors.requestTemperatures();
  float temp = sensors.getTempFByIndex(0);

  if(isSensorFailure(temp))
  {
    currentTempF = NAN;
    temperatureText = "Sensor Failure";
    updateTemperatureStatus();
    return;
  }

  currentTempF = temp;
  temperatureText = String(currentTempF, 2);
  updateTemperatureStatus();
}

// ============================================================================
// PREDICTION MODEL
// ============================================================================

void resetPredictionModel(const String& reason)
{
  sampleCount = 0;
  adaptiveK = -1.0f;
  predictedRemainingSeconds = -1.0f;
  modelSessionStartMillis = millis();
  modelDirection = getRequiredDirection(currentTempF, targetTempF);

  Serial.println();
  Serial.print("MODEL_RESET,");
  Serial.println(reason);
  Serial.print("MODEL_DIRECTION,");
  Serial.println(directionToString(modelDirection));
  Serial.print("TARGET_TEMP_F,");
  Serial.println(targetTempF, 2);
}

void captureModelSample()
{
  if(isSensorFailure(currentTempF))
  {
    return;
  }

  if(modelDirection == DIRECTION_NONE)
  {
    return;
  }

  if(sampleCount >= MAX_SAMPLES)
  {
    // Fifteen minutes of one-second samples is enough for the initial project.
    // A ring buffer can replace this later if longer sessions are needed.
    return;
  }

  temperatureSamples[sampleCount] = currentTempF;
  sampleTimes[sampleCount] = millis();
  sampleCount++;
}

float calculateRawK()
{
  if(sampleCount < 5 || modelDirection == DIRECTION_NONE)
  {
    return -1.0f;
  }

  float referenceTempF = getModelReferenceTemp(modelDirection);
  float sessionElapsed = (millis() - modelSessionStartMillis) / 1000.0f;

  if(sessionElapsed < MODEL_IGNORE_FIRST_SECONDS)
  {
    return -1.0f;
  }

  float earliestAllowedElapsed = sessionElapsed - MODEL_WINDOW_SECONDS;
  if(earliestAllowedElapsed < MODEL_IGNORE_FIRST_SECONDS)
  {
    earliestAllowedElapsed = MODEL_IGNORE_FIRST_SECONDS;
  }

  int firstIndex = -1;

  for(int i = 0; i < sampleCount; i++)
  {
    float elapsed = (sampleTimes[i] - modelSessionStartMillis) / 1000.0f;

    if(elapsed >= earliestAllowedElapsed)
    {
      firstIndex = i;
      break;
    }
  }

  if(firstIndex < 0 || sampleCount - firstIndex < 5)
  {
    return -1.0f;
  }

  unsigned long regressionStartMillis = sampleTimes[firstIndex];

  float sumTime = 0.0f;
  float sumLog = 0.0f;
  float sumTimeSquared = 0.0f;
  float sumTimeLog = 0.0f;
  int valid = 0;

  for(int i = firstIndex; i < sampleCount; i++)
  {
    float difference = fabs(temperatureSamples[i] - referenceTempF);

    // Prevent ln(0) and reject points essentially at the model reference.
    if(difference < 0.25f)
    {
      continue;
    }

    float t = (sampleTimes[i] - regressionStartMillis) / 1000.0f;
    float y = log(difference);

    sumTime += t;
    sumLog += y;
    sumTimeSquared += t * t;
    sumTimeLog += t * y;
    valid++;
  }

  if(valid < 5)
  {
    return -1.0f;
  }

  float denominator =
      valid * sumTimeSquared
      - sumTime * sumTime;

  if(fabs(denominator) < 0.000001f)
  {
    return -1.0f;
  }

  float slope =
      (valid * sumTimeLog - sumTime * sumLog)
      / denominator;

  float k = -slope;

  if(k <= 0.0f || isnan(k) || isinf(k))
  {
    return -1.0f;
  }

  return k;
}

float calculateRemainingTime(float k)
{
  if(k <= 0.0f || modelDirection == DIRECTION_NONE)
  {
    return (modelDirection == DIRECTION_NONE) ? 0.0f : -1.0f;
  }

  float referenceTempF = getModelReferenceTemp(modelDirection);

  float currentDifference = fabs(currentTempF - referenceTempF);
  float targetDifference = fabs(targetTempF - referenceTempF);

  // The target must lie between the current cabin temperature and the model's
  // effective equilibrium temperature for Newton-law prediction to make sense.
  if(targetDifference < 0.25f || currentDifference <= targetDifference)
  {
    return 0.0f;
  }

  float remaining = log(currentDifference / targetDifference) / k;

  if(remaining < 0.0f || isnan(remaining) || isinf(remaining))
  {
    return -1.0f;
  }

  return remaining;
}

void updatePredictionModel()
{
  ThermalDirection requiredDirection =
      getRequiredDirection(currentTempF, targetTempF);

  if(requiredDirection != modelDirection)
  {
    resetPredictionModel("THERMAL_DIRECTION_CHANGED");
    return;
  }

  if(requiredDirection == DIRECTION_NONE)
  {
    predictedRemainingSeconds = 0.0f;
    return;
  }

  float rawK = calculateRawK();

  if(rawK > 0.0f)
  {
    if(adaptiveK < 0.0f)
    {
      adaptiveK = rawK;
    }
    else
    {
      float minK = adaptiveK * (1.0f - MAX_K_CHANGE_FRACTION);
      float maxK = adaptiveK * (1.0f + MAX_K_CHANGE_FRACTION);

      rawK = clampFloat(rawK, minK, maxK);

      adaptiveK =
          K_SMOOTH_OLD_WEIGHT * adaptiveK
          + K_SMOOTH_NEW_WEIGHT * rawK;
    }
  }

  predictedRemainingSeconds = calculateRemainingTime(adaptiveK);

  Serial.print("MODEL,");
  Serial.print(adaptiveK, 8);
  Serial.print(",");
  Serial.println(predictedRemainingSeconds, 1);
}

// ============================================================================
// HVAC DECISION LOGIC
// ============================================================================

HVACCommand calculateHVACCommand()
{
  HVACCommand command;
  command.active = false;
  command.setpointF = targetTempF;
  command.fanLevel = 0;
  command.acOn = false;
  command.recirculationOn = false;
  command.mode = "AUTO OFF";

  if(!autoControlEnabled)
  {
    return command;
  }

  if(isSensorFailure(currentTempF))
  {
    command.mode = "SENSOR ERROR";
    return command;
  }

  command.active = true;

  float error = currentTempF - targetTempF;
  float magnitude = fabs(error);

  // -------------------------------------------------------------------------
  // TARGET MAINTENANCE
  // -------------------------------------------------------------------------
  if(magnitude <= TARGET_TOLERANCE_F)
  {
    command.setpointF = targetTempF;
    command.fanLevel = 1;
    command.acOn = false;
    command.recirculationOn = false;
    command.mode = "MAINTAINING TARGET";
    return command;
  }

  // -------------------------------------------------------------------------
  // COOLING
  // -------------------------------------------------------------------------
  if(error > 0.0f)
  {
    command.acOn = true;
    command.recirculationOn = true;

    if(magnitude > 12.0f)
    {
      command.setpointF = clampFloat(targetTempF - 8.0f, 60.0f, targetTempF);
      command.fanLevel = 7;
      command.mode = "RAPID COOLING";
    }
    else if(magnitude > 6.0f)
    {
      command.setpointF = clampFloat(targetTempF - 5.0f, 60.0f, targetTempF);
      command.fanLevel = 5;
      command.mode = "COOLING";
    }
    else if(magnitude > 2.0f)
    {
      command.setpointF = clampFloat(targetTempF - 2.0f, 60.0f, targetTempF);
      command.fanLevel = 3;
      command.mode = "APPROACHING TARGET";
    }
    else
    {
      command.setpointF = targetTempF;
      command.fanLevel = 2;
      command.mode = "FINAL COOLING";
    }

    return command;
  }

  // -------------------------------------------------------------------------
  // HEATING
  // -------------------------------------------------------------------------
  command.acOn = false;
  command.recirculationOn = false;

  if(magnitude > 12.0f)
  {
    command.setpointF = clampFloat(targetTempF + 8.0f, targetTempF, 85.0f);
    command.fanLevel = 7;
    command.mode = "RAPID HEATING";
  }
  else if(magnitude > 6.0f)
  {
    command.setpointF = clampFloat(targetTempF + 5.0f, targetTempF, 85.0f);
    command.fanLevel = 5;
    command.mode = "HEATING";
  }
  else if(magnitude > 2.0f)
  {
    command.setpointF = clampFloat(targetTempF + 2.0f, targetTempF, 85.0f);
    command.fanLevel = 3;
    command.mode = "APPROACHING TARGET";
  }
  else
  {
    command.setpointF = targetTempF;
    command.fanLevel = 2;
    command.mode = "FINAL HEATING";
  }

  return command;
}

bool hvacCommandsMatch(const HVACCommand& a, const HVACCommand& b)
{
  return
      a.active == b.active
      && fabs(a.setpointF - b.setpointF) < 0.1f
      && a.fanLevel == b.fanLevel
      && a.acOn == b.acOn
      && a.recirculationOn == b.recirculationOn
      && a.mode == b.mode;
}

// ============================================================================
// PHYSICAL HVAC INTERFACE - BENCH MODE STUB
// ============================================================================

void applyHVACCommand(const HVACCommand& command)
{
  if(hvacCommandsMatch(command, lastAppliedHVAC))
  {
    return;
  }

  // For now, only publish the desired actuator state.
  //
  // PERSON 3 / VEHICLE-INTERFACE TODO:
  // Replace or extend this function with the final actuator layer, e.g.:
  //   setTemperatureActuator(command.setpointF);
  //   setFanActuator(command.fanLevel);
  //   setACActuator(command.acOn);
  //   setRecirculationActuator(command.recirculationOn);
  //
  // Keep the high-level controller above independent from whether the final
  // implementation uses servos, button emulation, or a later vehicle-bus layer.

  Serial.print("HVAC_COMMAND,");
  Serial.print(command.active ? 1 : 0);
  Serial.print(",");
  Serial.print(command.setpointF, 1);
  Serial.print(",");
  Serial.print(command.fanLevel);
  Serial.print(",");
  Serial.print(command.acOn ? 1 : 0);
  Serial.print(",");
  Serial.print(command.recirculationOn ? 1 : 0);
  Serial.print(",");
  Serial.println(command.mode);

  lastAppliedHVAC = command;
}

void updateHVACController()
{
  desiredHVAC = calculateHVACCommand();
  applyHVACCommand(desiredHVAC);
}

// ============================================================================
// SIMPLIFIED SERIAL LOGGING
// ============================================================================

void printStateLog()
{
  Serial.print("STATE,");
  Serial.print(millis());
  Serial.print(",");

  if(isSensorFailure(currentTempF))
  {
    Serial.print("nan");
  }
  else
  {
    Serial.print(currentTempF, 2);
  }

  Serial.print(",");
  Serial.print(targetTempF, 1);
  Serial.print(",");
  Serial.print(predictedRemainingSeconds, 1);
  Serial.print(",");
  Serial.print(desiredHVAC.mode);
  Serial.print(",");
  Serial.print(desiredHVAC.setpointF, 1);
  Serial.print(",");
  Serial.print(desiredHVAC.fanLevel);
  Serial.print(",");
  Serial.print(desiredHVAC.acOn ? 1 : 0);
  Serial.print(",");
  Serial.print(desiredHVAC.recirculationOn ? 1 : 0);
  Serial.print(",");
  Serial.println(autoControlEnabled ? 1 : 0);
}

// ============================================================================
// WEB INTERFACE
// ============================================================================

const char index_html[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>CarClimate</title>
  <style>
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Arial, sans-serif;
      background: #f4f6f8;
      color: #1f2937;
    }
    .container {
      max-width: 650px;
      margin: auto;
      padding: 24px 16px 40px;
    }
    h1 {
      text-align: center;
      margin: 8px 0 4px;
      font-size: 2rem;
    }
    .subtitle {
      text-align: center;
      color: #6b7280;
      margin-bottom: 24px;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 14px;
    }
    .card {
      background: white;
      border-radius: 18px;
      padding: 20px;
      box-shadow: 0 4px 14px rgba(0,0,0,0.07);
    }
    .wide { grid-column: 1 / -1; }
    .label {
      color: #6b7280;
      font-size: 0.82rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      margin-bottom: 8px;
    }
    .big {
      font-size: 2.4rem;
      font-weight: 700;
    }
    .medium {
      font-size: 1.35rem;
      font-weight: 700;
    }
    .controls {
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 18px;
    }
    button {
      border: 0;
      border-radius: 12px;
      padding: 12px 18px;
      font-size: 1rem;
      font-weight: 700;
      cursor: pointer;
    }
    .round {
      width: 48px;
      height: 48px;
      border-radius: 50%;
      padding: 0;
      font-size: 1.5rem;
    }
    .primary {
      width: 100%;
      margin-top: 12px;
      background: #111827;
      color: white;
    }
    .off {
      background: #e5e7eb;
      color: #111827;
    }
    .row {
      display: flex;
      justify-content: space-between;
      padding: 9px 0;
      border-bottom: 1px solid #eef0f2;
    }
    .row:last-child { border-bottom: 0; }
    .value { font-weight: 700; }
    .bench {
      margin-top: 14px;
      font-size: 0.82rem;
      color: #6b7280;
      text-align: center;
    }
    @media(max-width: 520px) {
      .grid { grid-template-columns: 1fr; }
      .wide { grid-column: auto; }
    }
  </style>
</head>
<body>
<div class="container">
  <h1>CarClimate</h1>
  <div class="subtitle">Predictive Automotive Climate Control</div>

  <div class="grid">
    <div class="card">
      <div class="label">Cabin Temperature</div>
      <div class="big"><span id="currentTemp">--</span>°F</div>
    </div>

    <div class="card">
      <div class="label">Time To Target</div>
      <div class="medium" id="prediction">Learning...</div>
    </div>

    <div class="card wide">
      <div class="label">Target Temperature</div>
      <div class="controls">
        <button class="round off" onclick="changeTarget(-1)">−</button>
        <div class="big"><span id="targetTemp">72</span>°F</div>
        <button class="round off" onclick="changeTarget(1)">+</button>
      </div>
      <button id="autoButton" class="primary" onclick="toggleAuto()">
        Enable Auto Climate
      </button>
    </div>

    <div class="card wide">
      <div class="label">Controller</div>
      <div class="row"><span>Mode</span><span class="value" id="mode">--</span></div>
      <div class="row"><span>HVAC Setpoint</span><span class="value"><span id="hvacSetpoint">--</span>°F</span></div>
      <div class="row"><span>Fan</span><span class="value" id="fan">--</span></div>
      <div class="row"><span>A/C</span><span class="value" id="ac">--</span></div>
      <div class="row"><span>Recirculation</span><span class="value" id="recirc">--</span></div>
      <div class="row"><span>Status</span><span class="value" id="status">--</span></div>
    </div>
  </div>

  <div class="bench">
    V0.1 bench mode: HVAC commands are calculated and logged, but the physical
    vehicle-actuator layer is intentionally not implemented yet.
  </div>
</div>

<script>
let state = { autoEnabled: false, targetTemp: 72 };

function formatSeconds(seconds) {
  if(seconds === null || seconds < 0 || Number.isNaN(seconds)) return "Learning...";
  if(seconds === 0) return "Target reached";
  let total = Math.max(0, Math.round(seconds));
  let min = Math.floor(total / 60);
  let sec = total % 60;
  return min + " min " + String(sec).padStart(2, "0") + " sec";
}

function updateUI(data) {
  state = data;

  document.getElementById("currentTemp").textContent =
    data.currentTemp === null ? "--" : Number(data.currentTemp).toFixed(1);

  document.getElementById("targetTemp").textContent = Number(data.targetTemp).toFixed(0);
  document.getElementById("prediction").textContent = formatSeconds(data.predictedSeconds);
  document.getElementById("mode").textContent = data.mode;
  document.getElementById("hvacSetpoint").textContent = Number(data.hvacSetpoint).toFixed(0);
  document.getElementById("fan").textContent = data.fanLevel > 0 ? data.fanLevel + "/7" : "--";
  document.getElementById("ac").textContent = data.acOn ? "ON" : "OFF";
  document.getElementById("recirc").textContent = data.recircOn ? "ON" : "OFF";
  document.getElementById("status").textContent = data.status;

  let button = document.getElementById("autoButton");
  button.textContent = data.autoEnabled ? "Disable Auto Climate" : "Enable Auto Climate";
}

function refreshState() {
  fetch("/api/state")
    .then(r => r.json())
    .then(updateUI)
    .catch(() => {});
}

function setTarget(value) {
  fetch("/api/target?value=" + encodeURIComponent(value))
    .then(() => refreshState());
}

function changeTarget(delta) {
  setTarget(Number(state.targetTemp) + delta);
}

function toggleAuto() {
  let enabled = state.autoEnabled ? 0 : 1;
  fetch("/api/auto?enabled=" + enabled)
    .then(() => refreshState());
}

refreshState();
setInterval(refreshState, 2000);
</script>
</body>
</html>
)HTML";

String buildStateJson()
{
  String json = "{";

  json += "\"currentTemp\":";
  if(isSensorFailure(currentTempF)) json += "null";
  else json += String(currentTempF, 2);

  json += ",\"targetTemp\":";
  json += String(targetTempF, 1);
  json += ",\"predictedSeconds\":";
  if(predictedRemainingSeconds < 0.0f) json += "null";
  else json += String(predictedRemainingSeconds, 1);

  json += ",\"autoEnabled\":";
  json += autoControlEnabled ? "true" : "false";
  json += ",\"mode\":\"";
  json += desiredHVAC.mode;
  json += "\"";
  json += ",\"hvacSetpoint\":";
  json += String(desiredHVAC.setpointF, 1);
  json += ",\"fanLevel\":";
  json += String(desiredHVAC.fanLevel);
  json += ",\"acOn\":";
  json += desiredHVAC.acOn ? "true" : "false";
  json += ",\"recircOn\":";
  json += desiredHVAC.recirculationOn ? "true" : "false";
  json += ",\"status\":\"";
  json += systemStatus;
  json += "\"";
  json += ",\"k\":";
  if(adaptiveK < 0.0f) json += "null";
  else json += String(adaptiveK, 8);
  json += "}";

  return json;
}

void configureWebServer()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(200, "text/html", index_html);
  });

  server.on("/api/state", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(200, "application/json", buildStateJson());
  });

  server.on("/api/target", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    if(!request->hasParam("value"))
    {
      request->send(400, "text/plain", "Missing value");
      return;
    }

    float requested = request->getParam("value")->value().toFloat();
    float newTarget = clampFloat(requested, MIN_TARGET_TEMP_F, MAX_TARGET_TEMP_F);

    if(fabs(newTarget - targetTempF) >= 0.1f)
    {
      targetTempF = newTarget;
      updateTemperatureStatus();
      resetPredictionModel("TARGET_CHANGED");
      updateHVACController();
    }

    request->send(200, "application/json", buildStateJson());
  });

  server.on("/api/auto", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    if(!request->hasParam("enabled"))
    {
      request->send(400, "text/plain", "Missing enabled");
      return;
    }

    String value = request->getParam("enabled")->value();
    autoControlEnabled = (value == "1" || value == "true" || value == "on");

    resetPredictionModel(autoControlEnabled ? "AUTO_ENABLED" : "AUTO_DISABLED");
    updateHVACController();

    request->send(200, "application/json", buildStateJson());
  });

  server.begin();
}

// ============================================================================
// SETUP
// ============================================================================

void setup()
{
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("====================================================");
  Serial.println("Predictive Automotive Climate Control System V0.1");
  Serial.println("====================================================");

  sensors.begin();
  readTemperature();

  WiFi.mode(WIFI_AP);

  bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD);

  if(apStarted)
  {
    Serial.print("Wi-Fi AP: ");
    Serial.println(AP_SSID);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());
  }
  else
  {
    Serial.println("Wi-Fi access point failed to start.");
  }

  configureWebServer();
  Serial.println("Web server started.");

  resetPredictionModel("SYSTEM_START");
  updateHVACController();

  lastTemperatureRead = millis();
  lastModelUpdate = millis();
  lastStateLog = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop()
{
  unsigned long now = millis();

  if(now - lastTemperatureRead >= TEMPERATURE_INTERVAL_MS)
  {
    readTemperature();

    ThermalDirection requiredDirection =
        getRequiredDirection(currentTempF, targetTempF);

    if(requiredDirection != modelDirection)
    {
      resetPredictionModel("THERMAL_DIRECTION_CHANGED");
    }

    captureModelSample();

    // Update remaining time every temperature sample using the latest accepted k.
    if(adaptiveK > 0.0f)
    {
      predictedRemainingSeconds = calculateRemainingTime(adaptiveK);
    }
    else if(requiredDirection == DIRECTION_NONE)
    {
      predictedRemainingSeconds = 0.0f;
    }

    updateHVACController();
    lastTemperatureRead = now;
  }

  if(now - lastModelUpdate >= MODEL_UPDATE_INTERVAL_MS)
  {
    updatePredictionModel();
    updateHVACController();
    lastModelUpdate = now;
  }

  if(now - lastStateLog >= STATE_LOG_INTERVAL_MS)
  {
    printStateLog();
    lastStateLog = now;
  }
}
