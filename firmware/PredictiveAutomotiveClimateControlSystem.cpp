// ==========================================================
// Predictive Automotive Climate Control System
// V1 - Cooling-Only Closed-Loop Climate Controller
//
// Purpose:
//   - Read cabin temperature from a DS18B20 once per second.
//   - Compare it with a fixed 75 F target.
//   - Choose a cooling level based on temperature error.
//   - Control one temperature-knob servo and two fan-button servos.
//   - Print status to Serial only.
//   - No Wi-Fi, website, data logger, or prediction model.
//
// IMPORTANT:
//   1) Servo angles below are PLACEHOLDERS. Calibrate them before
//      mechanically attaching the servos to the vehicle controls.
//   2) The three servos must use a separate ~5 V supply.
//   3) ESP32 GND and servo-power GND MUST be connected together.
//   4) Keep ACTUATORS_ENABLED false until the servo hardware is ready.
// ==========================================================

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>

// ==========================================================
// USER / SYSTEM SETTINGS
// ==========================================================

const float TARGET_TEMP_F = 75.0;
const float LOWER_COMFORT_F = 73.0;
const float UPPER_COMFORT_F = 77.0;

const unsigned long TEMP_READ_INTERVAL_MS = 1000;
const unsigned long MIN_CONTROL_CHANGE_INTERVAL_MS = 15000;

// Set true only after the servo hardware and external 5 V supply are ready.
const bool ACTUATORS_ENABLED = false;

// ==========================================================
// PINS
// ==========================================================

const int ONE_WIRE_BUS = 13;
const int TEMP_KNOB_SERVO_PIN = 18;
const int FAN_UP_SERVO_PIN     = 19;
const int FAN_DOWN_SERVO_PIN   = 21;

// ==========================================================
// TEMPERATURE SENSOR
// ==========================================================

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

float currentTempF = 0.0;
bool sensorValid = false;

// ==========================================================
// SERVO OBJECTS
// ==========================================================

Servo tempKnobServo;
Servo fanUpServo;
Servo fanDownServo;

// ==========================================================
// SERVO CALIBRATION - PLACEHOLDER ANGLES ONLY
// ==========================================================

const int TEMP_ANGLE_MAX_COLD = 25;
const int TEMP_ANGLE_COLD     = 45;
const int TEMP_ANGLE_COOL     = 65;
const int TEMP_ANGLE_MAINTAIN = 85;

const int FAN_UP_REST_ANGLE  = 20;
const int FAN_UP_PRESS_ANGLE = 45;

const int FAN_DOWN_REST_ANGLE  = 20;
const int FAN_DOWN_PRESS_ANGLE = 45;

const unsigned long BUTTON_PRESS_MS   = 350;
const unsigned long BUTTON_RELEASE_MS = 250;

// ==========================================================
// FAN STATE
// ==========================================================

const int MIN_FAN_LEVEL = 1;
const int MAX_FAN_LEVEL = 7;

int currentFanLevel = MAX_FAN_LEVEL;

// ==========================================================
// CLIMATE-CONTROL STATES
// ==========================================================

enum ClimateMode
{
  MAX_COOL,
  STRONG_COOL,
  MODERATE_COOL,
  GENTLE_COOL,
  MAINTAIN,
  TOO_COLD
};

enum TempKnobSetting
{
  KNOB_MAX_COLD,
  KNOB_COLD,
  KNOB_COOL,
  KNOB_MAINTAIN
};

struct ClimateCommand
{
  ClimateMode mode;
  int fanLevel;
  TempKnobSetting knobSetting;
};

ClimateCommand activeCommand =
{
  MAX_COOL,
  MAX_FAN_LEVEL,
  KNOB_MAX_COLD
};

bool commandInitialized = false;

// ==========================================================
// TIMING
// ==========================================================

unsigned long lastTemperatureReadMs = 0;
unsigned long lastControlChangeMs = 0;

// ==========================================================
// SENSOR
// ==========================================================

bool isSensorFailure(float tempF)
{
  return tempF <= -190.0;
}

bool readCabinTemperature()
{
  sensors.requestTemperatures();
  float temp = sensors.getTempFByIndex(0);

  if (isSensorFailure(temp))
  {
    sensorValid = false;
    return false;
  }

  currentTempF = temp;
  sensorValid = true;
  return true;
}

// ==========================================================
// TEXT HELPERS
// ==========================================================

const char* climateModeName(ClimateMode mode)
{
  switch (mode)
  {
    case MAX_COOL:      return "MAX_COOL";
    case STRONG_COOL:   return "STRONG_COOL";
    case MODERATE_COOL: return "MODERATE_COOL";
    case GENTLE_COOL:   return "GENTLE_COOL";
    case MAINTAIN:      return "MAINTAIN";
    case TOO_COLD:      return "TOO_COLD";
    default:            return "UNKNOWN";
  }
}

const char* knobSettingName(TempKnobSetting setting)
{
  switch (setting)
  {
    case KNOB_MAX_COLD: return "MAX_COLD";
    case KNOB_COLD:     return "COLD";
    case KNOB_COOL:     return "COOL";
    case KNOB_MAINTAIN: return "MAINTAIN";
    default:            return "UNKNOWN";
  }
}

// ==========================================================
// TEMPERATURE-KNOB SERVO
// ==========================================================

int knobSettingToAngle(TempKnobSetting setting)
{
  switch (setting)
  {
    case KNOB_MAX_COLD: return TEMP_ANGLE_MAX_COLD;
    case KNOB_COLD:     return TEMP_ANGLE_COLD;
    case KNOB_COOL:     return TEMP_ANGLE_COOL;
    case KNOB_MAINTAIN: return TEMP_ANGLE_MAINTAIN;
    default:            return TEMP_ANGLE_MAINTAIN;
  }
}

void setTemperatureKnob(TempKnobSetting setting)
{
  int angle = knobSettingToAngle(setting);

  Serial.print("ACTUATOR: TEMP_KNOB -> ");
  Serial.print(knobSettingName(setting));
  Serial.print(" (");
  Serial.print(angle);
  Serial.println(" deg)");

  if (ACTUATORS_ENABLED)
  {
    tempKnobServo.write(angle);
  }
}

// ==========================================================
// FAN BUTTON SERVOS
// ==========================================================

void pressFanUp()
{
  Serial.println("ACTUATOR: PRESS FAN+");

  if (!ACTUATORS_ENABLED)
  {
    return;
  }

  fanUpServo.write(FAN_UP_PRESS_ANGLE);
  delay(BUTTON_PRESS_MS);
  fanUpServo.write(FAN_UP_REST_ANGLE);
  delay(BUTTON_RELEASE_MS);
}

void pressFanDown()
{
  Serial.println("ACTUATOR: PRESS FAN-");

  if (!ACTUATORS_ENABLED)
  {
    return;
  }

  fanDownServo.write(FAN_DOWN_PRESS_ANGLE);
  delay(BUTTON_PRESS_MS);
  fanDownServo.write(FAN_DOWN_REST_ANGLE);
  delay(BUTTON_RELEASE_MS);
}

// ==========================================================
// ESTABLISH KNOWN FAN STATE
// ==========================================================

void synchronizeFanToMaximum()
{
  Serial.println();
  Serial.println("Synchronizing fan state to level 7...");

  for (int i = 0; i < 6; i++)
  {
    pressFanUp();
  }

  currentFanLevel = MAX_FAN_LEVEL;

  Serial.println("Logical fan state = 7");
  Serial.println();
}

// ==========================================================
// SET FAN LEVEL
// ==========================================================

void setFanLevel(int desiredLevel)
{
  desiredLevel = constrain(desiredLevel, MIN_FAN_LEVEL, MAX_FAN_LEVEL);

  if (desiredLevel == currentFanLevel)
  {
    return;
  }

  Serial.print("ACTUATOR: FAN ");
  Serial.print(currentFanLevel);
  Serial.print(" -> ");
  Serial.println(desiredLevel);

  while (currentFanLevel < desiredLevel)
  {
    pressFanUp();
    currentFanLevel++;
  }

  while (currentFanLevel > desiredLevel)
  {
    pressFanDown();
    currentFanLevel--;
  }
}

// ==========================================================
// COOLING-ONLY CONTROL LOGIC
//
// Target = 75 F
// Comfort band = 73..77 F
//
// > 85 F       : fan 7, max cold
// > 81..85 F   : fan 5, max cold
// > 78..81 F   : fan 3, cold
// > 77..78 F   : fan 2, cool
// 73..77 F     : fan 1, maintain
// < 73 F       : fan 1, maintain / minimum cooling
// ==========================================================

ClimateCommand calculateClimateCommand(float tempF)
{
  ClimateCommand command;

  if (tempF > TARGET_TEMP_F + 10.0)
  {
    command.mode = MAX_COOL;
    command.fanLevel = 7;
    command.knobSetting = KNOB_MAX_COLD;
  }
  else if (tempF > TARGET_TEMP_F + 6.0)
  {
    command.mode = STRONG_COOL;
    command.fanLevel = 5;
    command.knobSetting = KNOB_MAX_COLD;
  }
  else if (tempF > TARGET_TEMP_F + 3.0)
  {
    command.mode = MODERATE_COOL;
    command.fanLevel = 3;
    command.knobSetting = KNOB_COLD;
  }
  else if (tempF > UPPER_COMFORT_F)
  {
    command.mode = GENTLE_COOL;
    command.fanLevel = 2;
    command.knobSetting = KNOB_COOL;
  }
  else if (tempF >= LOWER_COMFORT_F)
  {
    command.mode = MAINTAIN;
    command.fanLevel = 1;
    command.knobSetting = KNOB_MAINTAIN;
  }
  else
  {
    // Cooling-only V1: do not actively heat; simply minimize cooling.
    command.mode = TOO_COLD;
    command.fanLevel = 1;
    command.knobSetting = KNOB_MAINTAIN;
  }

  return command;
}

// ==========================================================
// COMMAND COMPARISON
// ==========================================================

bool commandChanged(const ClimateCommand& a, const ClimateCommand& b)
{
  return
    a.mode != b.mode ||
    a.fanLevel != b.fanLevel ||
    a.knobSetting != b.knobSetting;
}

// ==========================================================
// APPLY COMMAND
// ==========================================================

void applyClimateCommand(const ClimateCommand& desired)
{
  setTemperatureKnob(desired.knobSetting);
  setFanLevel(desired.fanLevel);

  activeCommand = desired;
  commandInitialized = true;
  lastControlChangeMs = millis();
}

// ==========================================================
// SERIAL STATUS
// ==========================================================

void printSystemStatus(const ClimateCommand& desired, bool waitingForMovementWindow)
{
  float error = currentTempF - TARGET_TEMP_F;

  Serial.print("TEMP: ");
  Serial.print(currentTempF, 2);
  Serial.print(" F");

  Serial.print(" | TARGET: ");
  Serial.print(TARGET_TEMP_F, 1);
  Serial.print(" F");

  Serial.print(" | ERROR: ");
  if (error >= 0)
  {
    Serial.print("+");
  }
  Serial.print(error, 2);
  Serial.print(" F");

  Serial.print(" | DESIRED_MODE: ");
  Serial.print(climateModeName(desired.mode));

  Serial.print(" | DESIRED_FAN: ");
  Serial.print(desired.fanLevel);

  Serial.print(" | DESIRED_KNOB: ");
  Serial.print(knobSettingName(desired.knobSetting));

  if (commandInitialized)
  {
    Serial.print(" | ACTIVE_FAN: ");
    Serial.print(currentFanLevel);

    Serial.print(" | ACTIVE_MODE: ");
    Serial.print(climateModeName(activeCommand.mode));
  }

  if (waitingForMovementWindow)
  {
    Serial.print(" | ACTUATOR_WAIT");
  }

  if (!ACTUATORS_ENABLED)
  {
    Serial.print(" | SIMULATION");
  }

  Serial.println();
}

// ==========================================================
// SERVO SETUP
// ==========================================================

void setupServos()
{
  tempKnobServo.setPeriodHertz(50);
  fanUpServo.setPeriodHertz(50);
  fanDownServo.setPeriodHertz(50);

  tempKnobServo.attach(TEMP_KNOB_SERVO_PIN, 500, 2400);
  fanUpServo.attach(FAN_UP_SERVO_PIN, 500, 2400);
  fanDownServo.attach(FAN_DOWN_SERVO_PIN, 500, 2400);

  fanUpServo.write(FAN_UP_REST_ANGLE);
  fanDownServo.write(FAN_DOWN_REST_ANGLE);
  tempKnobServo.write(TEMP_ANGLE_MAINTAIN);

  delay(500);
}

// ==========================================================
// SETUP
// ==========================================================

void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("==============================================");
  Serial.println("Predictive Automotive Climate Control System");
  Serial.println("V1 - Cooling-Only Controller");
  Serial.println("==============================================");
  Serial.print("Target: ");
  Serial.print(TARGET_TEMP_F, 1);
  Serial.println(" F");
  Serial.print("Comfort range: ");
  Serial.print(LOWER_COMFORT_F, 1);
  Serial.print(" - ");
  Serial.print(UPPER_COMFORT_F, 1);
  Serial.println(" F");

  if (ACTUATORS_ENABLED)
  {
    Serial.println("Actuators: ENABLED");
    setupServos();
    synchronizeFanToMaximum();
  }
  else
  {
    Serial.println("Actuators: DISABLED (simulation mode)");
    Serial.println("Set ACTUATORS_ENABLED = true after hardware is ready.");
    currentFanLevel = MAX_FAN_LEVEL;
  }

  sensors.begin();

  // Initial reading and initial control decision.
  if (readCabinTemperature())
  {
    ClimateCommand initial = calculateClimateCommand(currentTempF);

    Serial.println();
    Serial.println("Initial control decision:");

    applyClimateCommand(initial);
    printSystemStatus(initial, false);
  }
  else
  {
    Serial.println("ERROR: DS18B20 sensor not detected.");
  }

  lastTemperatureReadMs = millis();
}

// ==========================================================
// LOOP
// ==========================================================

void loop()
{
  unsigned long now = millis();

  if (now - lastTemperatureReadMs >= TEMP_READ_INTERVAL_MS)
  {
    lastTemperatureReadMs = now;

    if (!readCabinTemperature())
    {
      Serial.println("SENSOR FAILURE: no HVAC command issued.");
      return;
    }

    ClimateCommand desired = calculateClimateCommand(currentTempF);

    bool changed =
      !commandInitialized ||
      commandChanged(desired, activeCommand);

    bool movementWindowOpen =
      (now - lastControlChangeMs >= MIN_CONTROL_CHANGE_INTERVAL_MS);

    bool waitingForMovementWindow =
      changed && !movementWindowOpen;

    if (changed && movementWindowOpen)
    {
      Serial.println();
      Serial.println("----- HVAC ADJUSTMENT -----");

      applyClimateCommand(desired);

      Serial.println("---------------------------");
    }

    printSystemStatus(desired, waitingForMovementWindow);
  }
}
