# Immediate Team Tasks After V0.1

## Person 1 - Thermal model / firmware integration
- Verify target-temperature prediction using a configurable effective reference temperature.
- Calibrate cooling reference temperature from real car data.
- Confirm adaptive k remains stable while fan commands change.
- Decide whether fan-specific model parameters are needed.

## Person 2 - HVAC controller / UI
- Tune temperature-error thresholds for fan levels 1-7.
- Decide how aggressively setpoint should differ from the user's target.
- Improve web UI after the controller behavior is stable.
- Add manual/automatic test controls if useful.

## Person 3 - Physical vehicle interface
- Inspect the car's actual fan and temperature controls.
- Prototype a non-destructive actuator for one control first.
- Add implementation behind applyHVACCommand(), without changing the high-level controller.
- Establish a known actuator starting/calibration state.

## Person 4 - Testing / electrical integration
- Use the minimal session logger during integration tests.
- Define repeatable hot-cabin/cool-down tests.
- Track time-to-target, prediction error, overshoot, and command transitions.
- Document wiring and power requirements for actuator hardware.

## MVP for expo
- User selects target cabin temperature.
- ESP32 measures cabin temperature.
- System predicts time to target.
- Auto controller selects HVAC setpoint + fan speed.
- Hardware physically changes at least those two controls.
- Controller backs off near target.
- UI shows the full live state.
