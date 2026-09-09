# Changes from the solo V6.0 firmware

- Corrected the platform guard from the old `ESP33` branch to an explicit ESP32-only build.
- Replaced the hard-coded 68-72 F "ideal" band with a user-selected `targetTempF` and a 1 F tolerance.
- Separated the user target temperature from the Newton-model effective reference temperature.
- Removed the old 30/45/60-second model-comparison arrays and experiment-completion machinery.
- Replaced them with one adaptive live model intended for an operating climate controller.
- Increased temperature history to up to 15 minutes at one sample per second.
- Added cooling/heating direction handling.
- Added an `HVACCommand` structure containing desired setpoint, fan level, A/C, recirculation, and controller mode.
- Added a first-pass automatic state controller for rapid cooling/heating, approach, and target maintenance.
- Added `applyHVACCommand()` as the boundary between software decisions and future physical actuators.
- Added a new local web UI with target-temperature controls, Auto Climate enable/disable, prediction, and desired HVAC state.
- Replaced the old experiment-oriented serial output with a single `STATE` record plus HVAC/model diagnostic messages.
- Added a minimal Python session logger for integration testing.
