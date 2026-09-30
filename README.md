# Automotive Climate Control System

An ESP32-based automotive HVAC automation project that reads vehicle cabin temperature and uses servo motors to physically adjust HVAC controls.

## Project Goal

The goal is to build a simple closed-loop climate-control system for a vehicle that can:

- Read cabin temperature using a DS18B20 sensor
- Compare the measured temperature with a target temperature
- Select an appropriate cooling level
- Control the vehicle HVAC temperature knob and fan buttons using servo motors
- Reduce cooling as the cabin approaches the target temperature
- Maintain the cabin near the desired temperature

The first version is focused on **cooling only**.

## Current Status

### Completed

- Built and tested the ESP32 control circuit
- Verified DS18B20 temperature input
- Verified independent control of all three servo motors
- Verified the external servo power setup and shared ground
- Added temperature-based HVAC control logic in C++
- Added a 15-second minimum interval between HVAC adjustments
- Defined the first cooling-control strategy and fan-level logic

### In Progress

- Measuring the vehicle HVAC controls
- Designing CAD mounts for the temperature-knob and fan-button servos
- Designing servo adapters / button pushers
- Preparing the system for physical in-vehicle integration

### Next Steps

1. Finish CAD measurements and servo-mount designs
2. 3D print the first mount prototypes
3. Mechanically attach and calibrate the temperature-knob servo
4. Mechanically attach and calibrate the fan-up and fan-down servos
5. Enable and calibrate actuator commands in the integrated firmware
6. Perform initial in-vehicle tests
7. Tune temperature thresholds and servo behavior
8. Move the working breadboard circuit to a soldered prototyping board
9. Repeat vehicle tests and document final system performance

## Current Hardware

- ESP32
- DS18B20 temperature sensor
- 3 MG90S metal-gear micro servos
  - 1 temperature-knob servo
  - 1 fan-up button servo
  - 1 fan-down button servo
- External 5 V servo power source
- 1000 uF electrolytic capacitor
- USB power wiring
- Breadboard and jumper wires
- Resistors and supporting components

## Firmware

The current firmware is written in C++ for the ESP32.

Libraries:

- Arduino
- OneWire
- DallasTemperature
- ESP32Servo

The controller currently:

- Reads the DS18B20 once per second
- Uses a fixed target temperature of 75 F
- Uses a 73 F to 77 F comfort range
- Selects a cooling level based on temperature error
- Tracks the desired fan level
- Selects a temperature-knob setting
- Limits HVAC adjustments to no more than once every 15 seconds

The main integrated firmware currently keeps physical actuators disabled by default while the mechanical mounts and calibration are still being developed. Servo actuation has been bench-tested separately.

## Initial Control Strategy

| Cabin Temperature | Mode | Fan Level | HVAC Temperature |
| --- | --- | ---: | --- |
| Above 85 F | Maximum Cooling | 7 | Maximum Cold |
| 81-85 F | Strong Cooling | 5 | Maximum Cold |
| 78-81 F | Moderate Cooling | 3 | Cold |
| 77-78 F | Gentle Cooling | 2 | Cool |
| 73-77 F | Maintain | 1 | Maintain |
| Below 73 F | Too Cold | 1 | Minimum Cooling |

These are starting values and will be tuned after vehicle testing.

## Mechanical Design

The next phase of the project is the physical interface between the servos and the vehicle HVAC controls.

Planned CAD parts include:

- Temperature-knob servo mount
- Knob-to-servo coupler / adapter
- Fan-up button servo mount
- Fan-down button servo mount
- Servo button pushers
- Optional electronics enclosure

The first CAD iteration will prioritize reliable geometry, non-destructive mounting, and easy adjustment.

## Repository Structure

```text
Automotive-Climate-Control-System/
├── cad/
│   ├── source/
│   └── stl/
├── docs/
├── firmware/
├── hardware/
│   ├── components/
│   ├── protoboard/
│   └── wiring/
├── images/
└── README.md
```

## Project Background

This project is a follow-up to my earlier **Car Cabin Temperature Prediction System**.

The earlier project focused on temperature sensing, thermal modeling, prediction, and repeated vehicle testing. This project shifts toward embedded control, electronics, servo actuation, CAD, and electromechanical integration.

## Current Development Stage

The electronics and individual actuator functions are working on the bench. The project has **not yet completed mechanical installation or full closed-loop in-vehicle validation**.

The current development focus is CAD mounting, mechanical integration, and vehicle testing.

## Future Improvements

Possible later additions include:

- User-selectable target temperature
- Temperature prediction integration
- Rate-of-change-based control
- More advanced closed-loop control
- Web interface
- Data logging and performance analysis
- Additional HVAC controls
- Improved permanent electronics packaging

## Disclaimer

This is an experimental prototype intended for educational and engineering-development purposes. The system should be tested carefully and must not interfere with safe vehicle operation.
