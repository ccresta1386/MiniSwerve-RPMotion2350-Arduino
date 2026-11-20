// SPDX-FileCopyrightText: 2026 Robot project contributors
//
// SPDX-License-Identifier: MIT

#ifndef ROBOT_DCMOTOR_H
#define ROBOT_DCMOTOR_H

#include <Arduino.h>

namespace robot {

class DCMotor {
 public:
  DCMotor(uint8_t pin_a,
          uint8_t pin_b,
          float vref = 5.0f,
          uint32_t pwm_freq_hz = 10000,
          bool inverted = false);

  bool begin();
  void setVoltage(float voltage);
  void coast();
  void stop();
  void setDuty(uint16_t duty_u16);
  bool available() const;

 private:
  void writeDuty(uint16_t duty_a_u16, uint16_t duty_b_u16);
  uint8_t dutyToAnalogWrite(uint16_t duty_u16) const;

  uint8_t pin_a_;
  uint8_t pin_b_;
  float vref_;
  uint32_t pwm_freq_hz_;
  bool inverted_;
  bool available_;
};

}  // namespace robot

#endif  // ROBOT_DCMOTOR_H
