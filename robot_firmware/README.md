# Robot firmware

Open [`robot_firmware.ino`](robot_firmware.ino) in Arduino IDE and follow the
[Arduino upload guide](../docs/arduino-upload.md). This sketch runs the
swerve drive and reads the USB host gamepad.

## Purpose

It still defaults to decoded button-list serial output:
- `BUTTONS`: mapped pressed buttons, printed about once per second.
- Currently mapped: `A`, `B`, `X`, `Y`, `LB`, `RB`.

When `robot_config::kUsbHostMapToDrive` is true:
- Hold `RB` to enable drive commands.
- Press `A` to stop motor outputs.
- Hold `B` for the buzzer horn.
- Press `Y` to zero the steering encoders and save the offsets to EEPROM.
  The sketch restores valid saved offsets at boot.
- Left stick Y maps to `vx`.
- Left stick X maps to `vy`.
- Right stick X maps to rotation.
- USB disconnects stop motor outputs. A command expires 250 ms after the
  last valid gamepad report, even if the controller remains mounted.
- Releasing `RB` stops steer and drive motor outputs instead of holding the last
  steering target.
- Hold local GP21 during boot to run startup autotune. The current static
  config enables steer autotune and leaves drive autotune disabled.

Optional verbose output is still available:
- `REPORT`: raw HID bytes (hex), including report ID.
- `DELTA`: exactly which byte indices changed.
- `BUTTON_EVENT`: debounced byte/bit transitions for button mapping.
- `SUMMARY`: periodic min/max/span activity for byte discovery.
- `STATE`: optional compact axes output.

By default, all report IDs are processed.

Use the serial probe output first to verify the controller mapping before
letting the robot move under power.

## USB host reconnects

The Pico is the USB host for the gamepad or its dongle.

- Handles hotplug when the controller exposes multiple HID interfaces.
- Polls TinyUSB mounted state and prints `UNMOUNT missed-callback ...` if a disconnect callback is missed.
- Keeps the host running while unplugged and waits for the next `MOUNT`.
- Plays one rising-and-falling buzzer sweep when the gamepad or dongle disconnects.
- Retries HID receives when a mounted endpoint is ready.
- Logs `USB host: controller mounted, but no recent valid gamepad reports.`
  after about three seconds without usable input.

## Troubleshooting

- `usbh_helper.h: No such file`: open the sketch from its complete folder.
- `Adafruit_TinyUSB.h: No such file`: install `Adafruit TinyUSB Library`.
- `pio_usb.h: No such file`: install `Pico PIO USB` library.
- Host never enumerates. Check the board-specific USB data pins and VBUS power. The GP24/GP25 defaults require compatible robot hardware.

## Mapping Workflow

1. Keep controller at rest; confirm `BUTTONS none`.
2. Press and hold one mapped button; confirm it appears in the `BUTTONS` line.
3. Release it and move to the next button.
4. Repeat for all controls.

Recommended with current defaults:
1. Leave sticks untouched.
2. Press and release one button at a time.
3. Share any button that does not appear correctly.

For your wireless dongle, the sketch suppresses mirrored duplicate frames where
only byte `0` flips (`rid=0x04` vs `rid=0x00`) and payload bytes are identical.

`SUMMARY` prints top bytes by activity:
- `min/max/span` helps identify analog bytes.
- `toggles` bitmask helps identify button bits.

## Throttling Controls

These constants are at the top of `robot_firmware.ino`:
- `LOG_BUTTON_LIST_1HZ` (default `true`): print only the decoded pressed-button list.
- `BUTTON_LIST_LOG_INTERVAL_MS` (default `1000`): button-list sample cadence.
- `LOG_COMPACT_STATE` (default `false`): compact mode.
- `COMPACT_LOG_MIN_INTERVAL_MS` (default `40`): minimum spacing for movement logs.
- `COMPACT_IDLE_HEARTBEAT_MS` (default `1500`): keepalive while idle.
- `AXIS_MIN_DELTA_FOR_LOG` (default `3`): filters tiny jitter.
- `LOG_BUTTON_BYTES` (default `false`): includes bytes `23..26` in `STATE`.
- `LOG_BUTTON_EVENTS` (default `false`): emit debounced button transition events.
- `REPORT_ID_FILTER_ENABLED` (default `false`): process only one report ID.
- `REPORT_ID_FILTER` (default `0x04`): selected report ID.
- `BUTTON_EVENT_BYTE_FIRST/LAST` (default `13..31`): scanned byte window.
- `BUTTON_EVENT_STABLE_REPORTS` (default `1`): debounce depth per byte value.
- `BUTTON_EVENT_MIN_INTERVAL_MS` (default `0`): minimum spacing between event prints.
- `BUTTON_EVENT_IGNORE_FULLBYTE_FLIPS` (default `false`): suppresses `0x00<->0xFF` whole-byte toggles.
- `BUTTON_EVENT_MAX_TOGGLED_BITS` (default `2`): ignore changes that flip too many bits at once.
- `BUTTON_EVENT_SKIP_AXIS_BYTES` (default `true`): ignores bytes `9..12`.
- `BUTTON_EVENT_SKIP_NOISY_RANGE` + `BUTTON_EVENT_NOISY_FIRST/LAST` (default `false`, `15..22`): optional noisy-byte exclusion.
- `LOG_RAW_REPORTS` / `LOG_DELTA_LINES` (default `false`): full verbose debug.
- `STATS_INTERVAL_MS` (default `8000`): summary cadence.
- `HOST_REPORT_REQUEUE_INTERVAL_MS` (default `100`): periodically requeue HID reports after hotplug.
- `HOST_MOUNT_SETTLE_MS` (default `300`): wait briefly after mount before queuing reports.
- `HOST_NO_ACTIVE_REPORT_LOG_MS` (default `3000`): delay and log cadence while mounted without recent valid gamepad reports.
- `HOST_TASK_TIMEOUT_MS` (default `2`): host task poll timeout.

## Next Step (after mapping)

Before running under load:
1. Verify motor direction with wheels off the ground.
2. Verify each absolute encoder angle increases in the expected direction.
3. Confirm `RB` must be held for movement and `A` stops outputs.
4. After pressing `Y`, check for `saved-eeprom` in Serial Monitor. Reboot and
   check for `loaded-eeprom`. The sketch uses the offsets in `robot_config.cpp`
   until a valid EEPROM record is saved.
5. Tune gains and PWM behavior for the selected board core.

## Startup Autotune

Hold the GP21 select button while the board boots. The sketch samples GP21 with
`INPUT_PULLUP`; the button must pull the pin low for about 350 ms during setup.
Serial output starts with `Startup autotune requested by GP21 hold.` when it is
accepted, followed by an `AUTOTUNE ...` summary.
