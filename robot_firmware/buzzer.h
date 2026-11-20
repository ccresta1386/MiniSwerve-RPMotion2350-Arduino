// SPDX-FileCopyrightText: 2026 Robot project contributors
//
// SPDX-License-Identifier: MIT

#ifndef ROBOT_BUZZER_H
#define ROBOT_BUZZER_H

#include <Arduino.h>

namespace robot {

class Buzzer {
 public:
  explicit Buzzer(uint8_t pin, float volume = 0.1f);

  bool begin();
  void beep(uint16_t freq_hz = 1200, uint16_t duration_ms = 80);
  void on(uint16_t freq_hz = 1200);
  void off();

 private:
  uint8_t pin_;
  uint8_t duty_;
  bool available_;
};

}  // namespace robot

#endif  // ROBOT_BUZZER_H
