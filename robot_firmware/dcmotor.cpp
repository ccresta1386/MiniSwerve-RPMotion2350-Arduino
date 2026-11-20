// SPDX-FileCopyrightText: 2026 Robot project contributors
//
// SPDX-License-Identifier: MIT

#include "dcmotor.h"

#include <math.h>

namespace robot {

DCMotor::DCMotor(uint8_t pin_a,
                 uint8_t pin_b,
                 float vref,
                 uint32_t pwm_freq_hz,
                 bool inverted)
    : pin_a_(pin_a),
      pin_b_(pin_b),
      vref_(vref > 0.0f ? vref : 5.0f),
      pwm_freq_hz_(pwm_freq_hz),
      inverted_(inverted),
      available_(false) {}

bool DCMotor::begin() {
  pinMode(pin_a_, OUTPUT);
  pinMode(pin_b_, OUTPUT);

  available_ = true;
  stop();
  return true;
}

void DCMotor::setVoltage(float voltage) {
  if (!available_) {
    return;
  }

  float v = voltage;
  if (v > vref_) {
    v = vref_;
  } else if (v < -vref_) {
    v = -vref_;
  }

  uint16_t duty = static_cast<uint16_t>(lroundf((fabsf(v) / vref_) * 65535.0f));
  if (v >= 0.0f) {
    if (inverted_) {
      writeDuty(0, duty);
    } else {
      writeDuty(duty, 0);
    }
  } else {
    if (inverted_) {
      writeDuty(duty, 0);
    } else {
      writeDuty(0, duty);
    }
  }
}

void DCMotor::coast() {
  stop();
}

void DCMotor::stop() {
  writeDuty(0, 0);
}

void DCMotor::setDuty(uint16_t duty_u16) {
  if (!available_) {
    return;
  }
  if (inverted_) {
    writeDuty(0, duty_u16);
  } else {
    writeDuty(duty_u16, 0);
  }
}

bool DCMotor::available() const {
  return available_;
}

void DCMotor::writeDuty(uint16_t duty_a_u16, uint16_t duty_b_u16) {
  analogWrite(pin_a_, dutyToAnalogWrite(duty_a_u16));
  analogWrite(pin_b_, dutyToAnalogWrite(duty_b_u16));
}

uint8_t DCMotor::dutyToAnalogWrite(uint16_t duty_u16) const {
  return static_cast<uint8_t>((static_cast<uint32_t>(duty_u16) * 255u) / 65535u);
}

}  // namespace robot
