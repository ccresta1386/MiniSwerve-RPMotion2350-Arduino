// SPDX-FileCopyrightText: 2026 Robot project contributors
//
// SPDX-License-Identifier: MIT

#include "buzzer.h"

namespace robot {

Buzzer::Buzzer(uint8_t pin, float volume)
    : pin_(pin), duty_(0), available_(false) {
  if (volume < 0.0f) {
    volume = 0.0f;
  } else if (volume > 1.0f) {
    volume = 1.0f;
  }
  duty_ = static_cast<uint8_t>(volume * 127.0f);
}

bool Buzzer::begin() {
  pinMode(pin_, OUTPUT);
  off();
  available_ = true;
  return true;
}

void Buzzer::beep(uint16_t freq_hz, uint16_t duration_ms) {
  if (!available_) {
    return;
  }
  on(freq_hz);
  delay(duration_ms);
  off();
}

void Buzzer::on(uint16_t freq_hz) {
  if (!available_) {
    return;
  }
  (void)duty_;
  tone(pin_, freq_hz);
}

void Buzzer::off() {
  noTone(pin_);
  digitalWrite(pin_, LOW);
}

}  // namespace robot
