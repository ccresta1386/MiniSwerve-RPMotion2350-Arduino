// SPDX-FileCopyrightText: 2026 Robot project contributors
//
// SPDX-License-Identifier: MIT

#ifndef ROBOT_SMART_MOTOR_H
#define ROBOT_SMART_MOTOR_H

#include <Arduino.h>
#include <math.h>

#include "dcmotor.h"
#include "encoder.h"

namespace robot {

class PID {
 public:
  PID(float kp = 0.0f,
      float ki = 0.0f,
      float kd = 0.0f,
      float integrator_limit = -1.0f,
      float out_limit = -1.0f);

  void reset();
  float compute(float error, float dt_s);

  float kp;
  float ki;
  float kd;
  float integrator;
  float integrator_limit;
  float out_limit;

 private:
  bool have_last_error_;
  float last_error_;
};

struct VelocityAutotuneResult {
  bool ok;
  const char* error;
  float baseline_velocity;
  float steady_velocity;
  float delta_velocity;
  float plant_gain_k;
  float time_constant_s;
  float suggested_kp;
  float suggested_ki;
  float suggested_kd;
  float suggested_kv;
};

class SmartMotorController {
 public:
  enum Mode {
    MODE_VOLTAGE,
    MODE_VELOCITY,
    MODE_POSITION,
  };

  SmartMotorController(DCMotor* motor, Encoder* encoder, float vref = 3.3f);

  void setMode(Mode mode);
  void setMode(const char* mode);
  Mode mode() const;
  void setTarget(float value);
  float target() const;
  void update();

  void applyVelocityGains(float kp,
                          float ki,
                          float kd = 0.0f,
                          float kv = NAN,
                          float ka = NAN,
                          float ks = NAN);
  void setStaticFeedforward(float volts);
  void setVelocitySign(float sign);
  void setOutputSign(float sign);
  void setPositionVelocityLimit(float limit_deg_s);
  void clearPositionVelocityLimit();
  void setPositionFeedforwardEnabled(bool enabled);
  void setPositionDeadband(float deadband_deg);
  void setVoltageSlewRate(float volts_per_sec);
  void clearVoltageSlewRate();
  float lastVelocityDegS() const;

  VelocityAutotuneResult autotuneVelocity(float step_voltage = 1.0f,
                                          float settle_time_s = 0.5f,
                                          float measure_time_s = 2.0f,
                                          uint16_t sample_period_ms = 10);

  PID vel_pid;
  PID pos_pid;

 private:
  float clipVoltage(float voltage) const;
  float applySlew(float voltage, float dt_s);
  float velocityToVoltage(float vel_set, float vel_meas, float dt_s, float ff_ref = NAN);
  static float wrapAngleError(float error_deg);

  DCMotor* motor_;
  Encoder* encoder_;
  float vref_;
  Mode mode_;
  float target_;
  float vel_sign_;
  float output_sign_;
  float kv_;
  float ka_;
  float max_voltage_;
  bool has_pos_vel_limit_;
  float pos_vel_limit_;
  bool use_pos_feedforward_;
  float pos_deadband_;
  float static_ff_;
  bool has_voltage_slew_rate_;
  float voltage_slew_rate_;
  float last_cmd_voltage_;
  bool have_last_time_;
  uint32_t last_us_;
  float last_velocity_;
};

}  // namespace robot

#endif  // ROBOT_SMART_MOTOR_H
