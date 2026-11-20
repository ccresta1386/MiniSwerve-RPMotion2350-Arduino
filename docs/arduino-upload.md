# Upload the Arduino firmware

The uploadable sketch is `robot_firmware/robot_firmware.ino`. Open it
from the repository so Arduino IDE can compile the C++ files beside it.

## Install the board support and libraries

1. In Arduino IDE preferences, add the Arduino-Pico package index to
   **Additional Boards Manager URLs**. [Arduino-Pico installation guide](https://arduino-pico.readthedocs.io/en/stable/install.html)

   ```text
   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
   ```

2. In **Boards Manager**, install **Raspberry Pi Pico/RP2040/RP2350** by Earle F. Philhower, III.
3. In **Library Manager**, install **Adafruit TinyUSB Library** and
   **Pico PIO USB** by sekigon-gonnoc.

## Set the board options

Open the sketch and set the following options in **Tools**. This configuration
was build-tested with Arduino-Pico 5.5.1.

| Menu | Cytron Motion 2350 Pro setting |
| --- | --- |
| Board | Cytron Motion 2350 Pro |
| USB Stack | Adafruit TinyUSB |
| CPU Speed | 120 MHz |
| Flash Size | 2MB (no FS) |
| Upload Method | Default (UF2) |

Cytron specifies TinyUSB and 120 MHz for the onboard USB-A host port. The
firmware uses the board's GP24 (D+) and GP25 (D-) connections and stores
steering offsets in EEPROM. [Cytron USB host guide](https://www.cytron.io/tutorial/get-started-motion-2350-pro-arduinoide-usb-host-and-connect-to-remote-controller)

## Upload from Arduino IDE

1. Connect the board's **USB-C programming port** to your computer with a USB
   data cable, then switch the board on.
2. For the first Arduino upload, or if the serial port is missing, switch the
   board off. Hold **BOOT** while switching it back on, then release **BOOT**
   when the **RP2350** drive appears. In Arduino IDE, open **Tools > Port** and
   select **UF2 Board**. For later uploads, select the board's serial port.
   [Cytron Arduino IDE setup](https://sg.cytron.io/tutorial/get-started-motion-2350-pro-arduinoide-setting-up)
3. Click **Upload**. The IDE compiles the sketch, flashes it over USB, and the
   board reboots. Uploading the sketch replaces the firmware currently running
   on the board.
4. Select the serial port that appears after reboot and open **Serial Monitor**
   at **115200 baud**. If you missed the startup messages, reopen the monitor
   and reset the board. Look for `Robot hardware initialized.` and
   `USB host init complete`.
5. Plug the gamepad or dongle into the board's USB-A host port and check for
   `BUTTONS` output while pressing the mapped buttons.

## Upload a UF2 manually

If the IDE has trouble uploading, select **Sketch → Export Compiled Binary**.
Find the exported `.uf2` in the sketch's build folder. Repeat the **BOOT**
power-on sequence above and copy the UF2 to the **RP2350** drive.
The drive ejects when the board reboots. Then verify the serial output as
above. [Raspberry Pi BOOTSEL documentation](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html)

## Update the firmware

Edit the Arduino settings in `robot_firmware/robot_config.cpp` or
`robot_config.h`, then click **Upload** again.
