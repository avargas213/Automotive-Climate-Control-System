# Predictive Automotive Climate Control System

An ESP32-based automotive climate-control prototype that reads cabin temperature and automatically adjusts a vehicle's HVAC controls using servo motors.

## Project Goal

The goal of this project is to create a simple closed-loop climate-control system for a vehicle.

The system will:

- Read cabin temperature using a DS18B20 temperature sensor
- Compare the current temperature to a fixed target temperature
- Decide how much cooling is needed
- Use servo motors to physically control the vehicle's HVAC temperature knob and fan buttons
- Reduce cooling as the cabin approaches the target temperature
- Maintain the cabin near the target temperature

For the first version, the system is focused on **cooling only**.

## Current Target

- Target temperature: **75°F**
- Comfort range: **73°F to 77°F**
- Temperature reading interval: **1 second**
- Minimum time between HVAC adjustments: **15 seconds**

## Current Hardware

Already available:

- ESP32
- DS18B20 temperature sensor
- Breadboard
- Jumper wires
- Resistors and other basic components

Planned hardware:

- 3 MG90S metal-gear micro servos
  - 1 for the HVAC temperature knob
  - 1 for the fan-up button
  - 1 for the fan-down button
- External 5V servo power source
- 1000 µF electrolytic capacitor
- USB-A power pigtail
- 3D-printed servo mounts and adapters

## Software

The current firmware is written in C++ for the ESP32.

Libraries:

- Arduino
- OneWire
- DallasTemperature
- ESP32Servo

The current firmware does not use:

- Wi-Fi
- A web interface
- A data logger
- A temperature prediction model

These features may be added later if they improve the final system.

## Control Logic

The controller reads the cabin temperature and selects an HVAC setting based on how far the cabin is from the 75°F target.

Initial control strategy:

| Cabin Temperature | Mode | Fan Level | HVAC Temperature |
| --- | --- | ---: | --- |
| Above 85°F | Maximum Cooling | 7 | Maximum Cold |
| 81–85°F | Strong Cooling | 5 | Maximum Cold |
| 78–81°F | Moderate Cooling | 3 | Cold |
| 77–78°F | Gentle Cooling | 2 | Cool |
| 73–77°F | Maintain | 1 | Maintain |
| Below 73°F | Too Cold | 1 | Minimum Cooling |

These values are starting points and will be adjusted after real vehicle testing.

## Fan Control

The vehicle fan is controlled using two servo motors that press the physical fan-up and fan-down buttons.

Because the ESP32 does not directly know the vehicle's current fan level, the system will first synchronize the fan state by pressing the fan-up button enough times to guarantee that the fan reaches its maximum level.

The software can then track later fan-level changes.

## Mechanical Design

Custom parts will be designed to mount the servos to the vehicle HVAC controls.

Planned CAD parts include:

- Temperature-knob servo mount
- Knob-to-servo adapter or coupler
- Fan-up button servo mount
- Fan-down button servo mount
- Button pushers
- Possible electronics enclosure

The first CAD models will focus only on correct dimensions and basic geometry.

## Repository Structure

```text
Predictive-Automotive-Climate-Control-System/
├── cad/
├── docs/
├── firmware/
├── hardware/
├── images/
└── README.md
```

## Current Status

### Completed

- Created project repository
- Defined V1 system architecture
- Defined cooling-only control strategy
- Created initial ESP32 firmware
- Added DS18B20 temperature sensing
- Added simulated servo-control logic
- Added 15-second actuator-change protection

### In Progress

- Testing firmware with the temperature sensor
- Measuring the vehicle HVAC controls
- Creating initial CAD models
- Preparing servo hardware

### Next Steps

1. Test the current firmware with the DS18B20
2. Measure the HVAC knob and buttons
3. Create simple MG90S reference geometry in CAD
4. Design initial servo mounts
5. Bench-test each servo when the hardware arrives
6. Calibrate servo rest, press, and knob angles
7. Integrate all three servos on a breadboard
8. Install the prototype in the vehicle
9. Tune the temperature-control thresholds
10. Perform repeated vehicle testing
11. Improve the mechanical design
12. Move the electronics to a cleaner permanent prototype if needed

## Future Improvements

Possible later additions include:

- User-selectable target temperature
- Web interface
- Temperature prediction
- Rate-of-change-based control
- More advanced closed-loop control
- Additional HVAC controls
- Improved data logging and validation

## Background

This project is a separate follow-up to an earlier **Car Cabin Temperature Prediction System** project.

The earlier project focused on temperature sensing, thermal modeling, prediction, and repeated vehicle testing. This project focuses more heavily on embedded control, electronics, servo actuation, CAD, and electromechanical integration.

## Disclaimer

This is an experimental prototype intended for educational and engineering-development purposes. The system should be tested carefully and should not interfere with safe vehicle operation.
