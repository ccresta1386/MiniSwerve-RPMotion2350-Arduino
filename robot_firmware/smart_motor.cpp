// SPDX-FileCopyrightText: 2026 Robot project contributors
//
// SPDX-License-Identifier: MIT

#include "smart_motor.h"

#include <math.h>
#include <string.h>

namespace robot {

PID::PID(float kp_in, float ki_in, float kd_in, float integrator_limit_in, float out_limit_in)
    : kp(kp_in),
      ki(ki_in),
      kd(kd_in),
      integrator(0.0f),
      integrator_limit(integrator_limit_in),
      out_limit(out_limit_in),
      have_last_error_(false),
      last_error_(0.0f) {}

void PID::reset() {
  integrator = 0.0f;
  have_last_error_ = false;
  last_error_ = 0.0f;
}

float PID::compute(float error, float dt_s) {
  if (dt_s <= 0.0f) {
    return 0.0f;
  }

  float p = kp * error;
  if (ki != 0.0f) {
    integrator += error * dt_s;
    if (integrator_limit >= 0.0f) {
      if (integrator > integrator_limit) {
        integrator = integrator_limit;
      } else if (integrator < -integrator_limit) {
        integrator = -integrator_limit;
      }
    }
  }

  float d = 0.0f;
  if (have_last_error_ && kd != 0.0f) {
    d = kd * ((error - last_error_) / dt_s);
  }
  have_last_error_ = true;
  last_error_ = error;

  float out = p + (ki * integrator) + d;
  if (out_limit >= 0.0f) {
    if (out > out_limit) {
      out = out_limit;
    } else if (out < -out_limit) {
      out = -out_limit;
    }
  }
  return out;
}

SmartMotorController::SmartMotorController(DCMotor* motor, Encoder* encoder, float vref)
    : vel_pid(0.0f, 0.0f, 0.0f, vref * 2.0f, vref),
      pos_pid(0.0f, 0.0f, 0.0f, 500.0f, 1000000.0f),
      motor_(motor),
      encoder_(encoder),
      vref_(vref > 0.0f ? vref : 3.3f),
      mode_(MODE_VOLTAGE),
      target_(0.0f),
      vel_sign_(1.0f),
      output_sign_(1.0f),
      kv_(0.0f),
      ka_(0.0f),
      max_voltage_(vref_),
      has_pos_vel_limit_(false),
      pos_vel_limit_(0.0f),
      use_pos_feedforward_(true),
      pos_deadband_(0.0f),
      static_ff_(0.0f),
      has_voltage_slew_rate_(false),
      voltage_slew_rate_(0.0f),
      last_cmd_voltage_(0.0f),
      have_last_time_(false),
      last_us_(0),
      last_velocity_(0.0f) {}

void SmartMotorController::setMode(Mode mode) {
  mode_ = mode;
  vel_pid.reset();
  pos_pid.reset();
}

void SmartMotorController::setMode(const char* mode) {
  if (strcmp(mode, "velocity") == 0) {
    setMode(MODE_VELOCITY);
  } else if (strcmp(mode, "position") == 0) {
    setMode(MODE_POSITION);
  } else {
    setMode(MODE_VOLTAGE);
  }
}

SmartMotorController::Mode SmartMotorController::mode() const {
  return mode_;
}

void SmartMotorController::setTarget(float value) {
  target_ = value;
}

float SmartMotorController::target() const {
  return target_;
}

void SmartMotorController::update() {
  uint32_t now_us = micros();
  if (!have_last_time_) {
    have_last_time_ = true;
    last_us_ = now_us;
    return;
  }

  uint32_t dt_us = now_us - last_us_;
  if (dt_us == 0) {
    return;
  }
  last_us_ = now_us;
  float dt_s = static_cast<float>(dt_us) / 1000000.0f;

  if (motor_ == nullptr || !motor_->available()) {
    return;
  }

  if (mode_ == MODE_VOLTAGE) {
    motor_->setVoltage(clipVoltage(target_ * output_sign_));
    return;
  }

  float vel = 0.0f;
  float angle = 0.0f;
  if (encoder_ != nullptr) {
    vel = vel_sign_ * encoder_->getAngularVelocityDegS();
    angle = encoder_->getAngleDeg();
  }
  last_velocity_ = vel;

  if (mode_ == MODE_VELOCITY) {
    motor_->setVoltage(velocityToVoltage(target_, vel, dt_s));
    return;
  }

  float pos_err = wrapAngleError(target_ - angle);
  if (pos_deadband_ > 0.0f && fabsf(pos_err) < pos_deadband_) {
    pos_err = 0.0f;
  }

  float pos_cmd = pos_pid.compute(pos_err, dt_s);
  if (has_pos_vel_limit_) {
    if (pos_cmd > pos_vel_limit_) {
      pos_cmd = pos_vel_limit_;
    } else if (pos_cmd < -pos_vel_limit_) {
      pos_cmd = -pos_vel_limit_;
    }
  }

  float ff = use_pos_feedforward_ ? kv_ * pos_cmd : 0.0f;
  float stat = 0.0f;
  if (static_ff_ != 0.0f && fabsf(pos_cmd) > 0.0f) {
    stat = pos_cmd > 0.0f ? static_ff_ : -static_ff_;
  }
  float vcmd = applySlew(clipVoltage(pos_cmd + ff + stat), dt_s);
  motor_->setVoltage(vcmd * output_sign_);
}

void SmartMotorController::applyVelocityGains(float kp,
                                              float ki,
                                              float kd,
                                              float kv,
                                              float ka,
                                              float ks) {
  vel_pid.kp = kp;
  vel_pid.ki = ki;
  vel_pid.kd = kd;
  if (!isnan(kv)) {
    kv_ = kv;
  }
  if (!isnan(ka)) {
    ka_ = ka;
  }
  if (!isnan(ks)) {
    static_ff_ = ks;
  }
}

void SmartMotorController::setStaticFeedforward(float volts) {
  static_ff_ = volts < 0.0f ? 0.0f : volts;
}

void SmartMotorController::setVelocitySign(float sign) {
  vel_sign_ = sign < 0.0f ? -1.0f : 1.0f;
}

void SmartMotorController::setOutputSign(float sign) {
  output_sign_ = sign < 0.0f ? -1.0f : 1.0f;
}

void SmartMotorController::setPositionVelocityLimit(float limit_deg_s) {
  has_pos_vel_limit_ = true;
  pos_vel_limit_ = fabsf(limit_deg_s);
}

void SmartMotorController::clearPositionVelocityLimit() {
  has_pos_vel_limit_ = false;
}

void SmartMotorController::setPositionFeedforwardEnabled(bool enabled) {
  use_pos_feedforward_ = enabled;
}

void SmartMotorController::setPositionDeadband(float deadband_deg) {
  pos_deadband_ = deadband_deg < 0.0f ? 0.0f : deadband_deg;
}

void SmartMotorController::setVoltageSlewRate(float volts_per_sec) {
  has_voltage_slew_rate_ = true;
  voltage_slew_rate_ = volts_per_sec < 0.0f ? 0.0f : volts_per_sec;
}

void SmartMotorController::clearVoltageSlewRate() {
  has_voltage_slew_rate_ = false;
}

float SmartMotorController::lastVelocityDegS() const {
  return last_velocity_;
}

VelocityAutotuneResult SmartMotorController::autotuneVelocity(float step_voltage,
                                                              float settle_time_s,
                                                              float measure_time_s,
                                                              uint16_t sample_period_ms) {
  struct Sample {
    float t_s;
    float velocity;
  };
  constexpr size_t kMaxSamples = 160;
  Sample samples[kMaxSamples];
  size_t sample_count = 0;

  VelocityAutotuneResult result = {};
  result.ok = false;
  result.error = nullptr;

  if (motor_ == nullptr || encoder_ == nullptr) {
    result.error = "missing motor or encoder";
    return result;
  }
  if (step_voltage <= 0.0f) {
    result.error = "step_voltage must be positive";
    return result;
  }
  if (step_voltage > vref_) {
    step_voltage = vref_;
  }

  motor_->setVoltage(0.0f);
  delay(200);

  float baseline = 0.0f;
  constexpr uint8_t kBaselineSamples = 5;
  for (uint8_t i = 0; i < kBaselineSamples; i++) {
    baseline += encoder_->getAngularVelocityDegS();
    delay(50);
  }
  baseline /= static_cast<float>(kBaselineSamples);

  uint32_t start_ms = millis();
  uint32_t record_start_ms = start_ms + static_cast<uint32_t>(settle_time_s * 1000.0f);
  uint32_t end_ms = record_start_ms + static_cast<uint32_t>(measure_time_s * 1000.0f);
  uint32_t last_sample_ms = 0;

  motor_->setVoltage(step_voltage);
  while ((int32_t)(millis() - end_ms) < 0) {
    uint32_t now_ms = millis();
    if ((int32_t)(now_ms - record_start_ms) >= 0 &&
        now_ms - last_sample_ms >= sample_period_ms) {
      last_sample_ms = now_ms;
      float vel = encoder_->getAngularVelocityDegS();
      if (sample_count < kMaxSamples) {
        samples[sample_count].t_s = static_cast<float>(now_ms - start_ms) / 1000.0f;
        samples[sample_count].velocity = vel;
        sample_count++;
      }
    }
    delay(1);
  }
  motor_->setVoltage(0.0f);

  if (sample_count == 0) {
    result.error = "no samples collected";
    return result;
  }

  size_t last_n = sample_count / 5;
  if (last_n < 1) {
    last_n = 1;
  }
  float steady = 0.0f;
  for (size_t i = sample_count - last_n; i < sample_count; i++) {
    steady += samples[i].velocity;
  }
  steady /= static_cast<float>(last_n);

  float delta_v = steady - baseline;
  if (fabsf(delta_v) < 1e-6f) {
    result.error = "no measurable velocity change";
    return result;
  }

  float plant_gain = delta_v / step_voltage;
  float threshold = baseline + (0.63f * delta_v);
  float tau = 1.0f;
  for (size_t i = 0; i < sample_count; i++) {
    float vel = samples[i].velocity;
    if ((delta_v > 0.0f && vel >= threshold) || (delta_v < 0.0f && vel <= threshold)) {
      tau = samples[i].t_s;
      break;
    }
  }
  result.ok = true;
  result.baseline_velocity = baseline;
  result.steady_velocity = steady;
  result.delta_velocity = delta_v;
  result.plant_gain_k = plant_gain;
  result.time_constant_s = tau;
  result.suggested_kp = 0.8f * (1.0f / plant_gain);
  result.suggested_ki = result.suggested_kp / (tau > 1e-6f ? tau : 1.0f);
  result.suggested_kd = 0.0f;
  result.suggested_kv = step_voltage / delta_v;
  return result;
}

float SmartMotorController::clipVoltage(float voltage) const {
  if (voltage > max_voltage_) {
    return max_voltage_;
  }
  if (voltage < -max_voltage_) {
    return -max_voltage_;
  }
  return voltage;
}

float SmartMotorController::applySlew(float voltage, float dt_s) {
  if (!has_voltage_slew_rate_ || dt_s <= 0.0f) {
    last_cmd_voltage_ = voltage;
    return voltage;
  }

  float max_step = voltage_slew_rate_ * dt_s;
  float delta = voltage - last_cmd_voltage_;
  if (delta > max_step) {
    voltage = last_cmd_voltage_ + max_step;
  } else if (delta < -max_step) {
    voltage = last_cmd_voltage_ - max_step;
  }
  last_cmd_voltage_ = voltage;
  return voltage;
}

float SmartMotorController::velocityToVoltage(float vel_set, float vel_meas, float dt_s, float ff_ref) {
  if (isnan(ff_ref)) {
    ff_ref = vel_set;
  }
  if (fabsf(vel_set) < 1e-3f && fabsf(vel_meas) < 2.0f) {
    vel_pid.reset();
  }

  float err = vel_set - vel_meas;
  float pid_out = vel_pid.compute(err, dt_s);
  float ff = kv_ * ff_ref;
  float stat = 0.0f;
  if (static_ff_ != 0.0f && fabsf(vel_set) > 0.0f) {
    stat = vel_set > 0.0f ? static_ff_ : -static_ff_;
  }
  float vcmd = applySlew(clipVoltage(pid_out + ff + stat), dt_s);
  if (fabsf(vcmd) < 0.02f) {
    vcmd = 0.0f;
  }
  return vcmd * output_sign_;
}

float SmartMotorController::wrapAngleError(float error_deg) {
  float wrapped = fmodf(error_deg + 180.0f, 360.0f);
  if (wrapped < 0.0f) {
    wrapped += 360.0f;
  }
  return wrapped - 180.0f;
}

}  // namespace robot
