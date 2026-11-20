// SPDX-FileCopyrightText: 2026 Robot project contributors
//
// SPDX-License-Identifier: MIT

#ifndef USBH_HELPER_H
#define USBH_HELPER_H

#include <Arduino.h>

// TinyUSB host support for RP2040/RP2350 using Pico-PIO-USB.
// This header is intentionally local to this sketch so it doesn't depend on
// vendor-provided example bundles.

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
  #include "pio_usb.h"
#endif

// Default D+ pin if board variant does not define one.
// Project target wiring uses D+=GP24 and D-=GP25.
#ifndef PIN_USB_HOST_DP
  #define PIN_USB_HOST_DP 24
#endif

// Optional VBUS enable pin:
// - If your board needs explicit VBUS control, define PIN_5V_EN and
//   PIN_5V_EN_STATE in your sketch or board variant.

#include "Adafruit_TinyUSB.h"

#if defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421
  #include "SPI.h"
  #if defined(ARDUINO_METRO_ESP32S2)
    Adafruit_USBH_Host USBHost(&SPI, 15, 14);
  #elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    Adafruit_USBH_Host USBHost(&SPI, 33, 15);
  #else
    Adafruit_USBH_Host USBHost(&SPI, 10, 9);
  #endif
#else
  // Native USB host for RP2040/RP2350.
  Adafruit_USBH_Host USBHost;
#endif

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
static void rp2040_configure_pio_usb(void) {
  Serial.println("Configuring USB host with TinyUSB and Pico-PIO-USB");

  #ifdef PIN_5V_EN
    pinMode(PIN_5V_EN, OUTPUT);
    #ifdef PIN_5V_EN_STATE
      digitalWrite(PIN_5V_EN, PIN_5V_EN_STATE);
    #else
      digitalWrite(PIN_5V_EN, HIGH);
    #endif
  #endif

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_USB_HOST_DP;

  #if defined(ARDUINO_RASPBERRY_PI_PICO_W)
    // Pico W shares PIO with cyw43 and needs alternate channels.
    pio_cfg.sm_tx = 3;
    pio_cfg.sm_rx = 2;
    pio_cfg.sm_eop = 3;
    pio_cfg.pio_rx_num = 0;
    pio_cfg.pio_tx_num = 1;
    pio_cfg.tx_ch = 9;
  #endif

  USBHost.configure_pio_usb(1, &pio_cfg);
}
#else
static void rp2040_configure_pio_usb(void) {
  // Non-RP2040/RP2350 target: no-op.
}
#endif

#endif  // USBH_HELPER_H

