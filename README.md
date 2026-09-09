# Predictive Automotive Climate Control System

## V0.1 Team Starter

This repository is intended as the starting point for a four-person extension of the **Car Cabin Temperature Prediction System**.

The original project was developed as a solo ESP32 system for sensing cabin temperature, estimating a Newton-law thermal response, and predicting time to an ideal temperature. This team project keeps that foundation while adding automatic climate-control decisions and a future physical interface to the vehicle HVAC controls.

## What V0.1 already does

- Reads cabin temperature from a DS18B20 on GPIO 13.
- Creates an ESP32 Wi-Fi access point named `CarClimate`.
- Hosts a local control/status page.
- Lets the user choose a target cabin temperature from 60-85 F.
- Lets the user enable/disable Auto Climate.
- Calculates desired HVAC temperature, fan level, A/C state, and recirculation state.
- Uses a simplified single adaptive Newton-law model to estimate remaining time to the selected target.
- Prints HVAC commands over Serial for bench testing.
- Provides a small optional serial logger for whole-system tests.

## What V0.1 intentionally does NOT do yet

It does not physically control a car.

`applyHVACCommand()` is currently an actuator abstraction/stub. The vehicle-interface subteam should later connect that function to the chosen physical actuator method after the controller is validated on the bench.

## Suggested firmware filename

`firmware/PredictiveAutomotiveClimateControlSystem.cpp`

## Suggested GitHub repository name

`Predictive-Automotive-Climate-Control-System`

## First bench test

1. Flash the firmware to the ESP32 using the same libraries as the original project.
2. Connect the DS18B20 to the existing GPIO 13 circuit.
3. Open Serial Monitor at 115200 baud.
4. Connect a phone/laptop to `CarClimate` using password `CarClimate123`.
5. Browse to the ESP32 AP IP shown in Serial Monitor (normally `192.168.4.1`).
6. Set a target temperature.
7. Enable Auto Climate.
8. Warm/cool the sensor by hand or with the same safe test setup used in the original project.
9. Confirm that `HVAC_COMMAND` changes as the temperature error changes.

## Important model note

The new prediction distinguishes:

- **User target temperature**: e.g. 72 F.
- **Effective HVAC model reference temperature**: the approximate equilibrium temperature used in the Newton model.

The starter values are 55 F while cooling and 95 F while heating. These are calibration placeholders. Real-car testing should determine better values or replace them with a model that depends on vent temperature/fan state.

## Recommended next implementation order

1. Verify V0.1 temperature sensing and UI.
2. Bench-test controller state changes without physical actuators.
3. Tune the controller thresholds and target tolerance.
4. Prototype one physical HVAC control actuator.
5. Implement temperature up/down control.
6. Implement fan up/down control.
7. Add A/C and recirculation only if time allows.
8. Run repeated real-car tests and calibrate the thermal model.

## Project lineage

When this becomes the team repository, add a direct link here to the original solo GitHub repository and name which V6.0 components were carried forward. Keep the original repository unchanged so the solo work and later team contributions remain clearly distinguishable.
