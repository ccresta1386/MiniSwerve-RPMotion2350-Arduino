# Arduino robot firmware

Open [`robot_firmware/robot_firmware.ino`](robot_firmware/robot_firmware.ino)
in Arduino IDE to build and upload the robot firmware. The sketch folder
contains all of its C++ source and headers. Follow the [Arduino upload guide](docs/arduino-upload.md).

| File | Purpose |
| --- | --- |
| `robot_firmware.ino` | USB host setup, gamepad input, and the swerve control loop |
| `robot_config.cpp`, `robot_config.h` | Pins, limits, gains, and EEPROM calibration |
| `swerve.*`, `smart_motor.*`, `dcmotor.*`, `encoder.*` | Drive control, motor outputs, and encoder feedback |
| `startup_autotune.*`, `buzzer.*`, `usbh_helper.h` | Startup tuning, buzzer, and USB host support |

See the [firmware README](robot_firmware/README.md) for controls and diagnostics.
