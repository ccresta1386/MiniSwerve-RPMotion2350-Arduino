// SPDX-FileCopyrightText: 2026 Robot project contributors
//
// SPDX-License-Identifier: MIT

#include "swerve.h"

#include <math.h>

namespace robot {
namespace {

constexpr float kPi = 3.14159265358979323846f;

float wrap360(float value) {
  float wrapped = fmodf(value, 360.0f);
  if (wrapped < 0.0f) {
    wrapped += 360.0f;
  }
  return wrapped;
}

float shortestAngleError(float target, float current) {
  float diff = fmodf(target - current + 180.0f, 360.0f);
  if (diff < 0.0f) {
    diff += 360.0f;
  }
  return diff - 180.0f;
}

}  // namespace

SwerveGains defaultSwerveGains() {
  SwerveGains gains;
  gains.drive_vel = {0.02f, 0.0f, 0.0f, 0.01f, 0.0f};
  gains.steer_pos = {1.5f, 0.0f, 0.0f, 0.0f, 0.0f};
  gains.steer_vel = {0.015f, 0.0f, 0.0f, 0.01f, 0.0f};
  return gains;
}

SwerveModule::SwerveModule(const SwerveModuleConfig& config,
                           DCMotor* drive_motor,
                           DCMotor* steer_motor,
                           Encoder* drive_encoder,
                           Encoder* steer_encoder,
                           float vref)
    : cfg_(config),
      wheel_radius_m_(defaultWheelDiameter(config.wheel_diameter_m) * 0.5f),
      drive_motor_(drive_motor),
      steer_motor_(steer_motor),
      drive_encoder_(drive_encoder),
      steer_encoder_(steer_encoder),
      drive_ctrl_(drive_motor, drive_encoder, vref),
      steer_ctrl_(steer_motor, steer_encoder, vref) {}

bool SwerveModule::begin(const SwerveGains& gains) {
  if (drive_encoder_ != nullptr) {
    drive_encoder_->begin();
  }
  if (steer_encoder_ != nullptr) {
    steer_encoder_->begin();
  }
  if (drive_motor_ != nullptr) {
    drive_motor_->begin();
  }
  if (steer_motor_ != nullptr) {
    steer_motor_->begin();
  }
  drive_ctrl_.setMode(SmartMotorController::MODE_VELOCITY);
  steer_ctrl_.setMode(SmartMotorController::MODE_POSITION);
  drive_ctrl_.setVelocitySign(cfg_.drive_vel_sign);
  drive_ctrl_.setOutputSign(cfg_.drive_output_sign);
  steer_ctrl_.setVelocitySign(cfg_.steer_vel_sign);
  steer_ctrl_.setOutputSign(cfg_.steer_output_sign);
  if (cfg_.has_steer_pos_vel_limit) {
    steer_ctrl_.setPositionVelocityLimit(cfg_.steer_pos_vel_limit);
  }
  steer_ctrl_.setPositionFeedforwardEnabled(cfg_.steer_pos_feedforward);
  steer_ctrl_.setPositionDeadband(cfg_.steer_pos_deadband);
  drive_ctrl_.setStaticFeedforward(cfg_.drive_static_ff);
  steer_ctrl_.setStaticFeedforward(cfg_.steer_static_ff);
  if (cfg_.drive_voltage_slew > 0.0f) {
    drive_ctrl_.setVoltageSlewRate(cfg_.drive_voltage_slew);
  }
  if (cfg_.steer_voltage_slew > 0.0f) {
    steer_ctrl_.setVoltageSlewRate(cfg_.steer_voltage_slew);
  }
  applyDefaultGains(gains);
  return true;
}

void SwerveModule::applyDefaultGains(const SwerveGains& gains) {
  drive_ctrl_.applyVelocityGains(gains.drive_vel.kp,
                                 gains.drive_vel.ki,
                                 gains.drive_vel.kd,
                                 gains.drive_vel.kv,
                                 NAN,
                                 gains.drive_vel.ks);
  steer_ctrl_.applyVelocityGains(gains.steer_vel.kp,
                                 gains.steer_vel.ki,
                                 gains.steer_vel.kd,
                                 gains.steer_vel.kv,
                                 NAN,
                                 gains.steer_vel.ks);
  steer_ctrl_.pos_pid.kp = gains.steer_pos.kp;
  steer_ctrl_.pos_pid.ki = gains.steer_pos.ki;
  steer_ctrl_.pos_pid.kd = gains.steer_pos.kd;
  if (gains.steer_pos.ks > 0.0f) {
    steer_ctrl_.setStaticFeedforward(gains.steer_pos.ks);
  }
}

float SwerveModule::zeroSteerHere() {
  if (steer_encoder_ == nullptr) {
    return NAN;
  }
  steer_encoder_->zero();
  return steer_encoder_->offsetDeg();
}

void SwerveModule::setState(float drive_velocity_m_s, float steer_angle_deg) {
  float current_angle = 0.0f;
  if (steer_encoder_ != nullptr) {
    current_angle = steer_encoder_->getAngleDeg();
  }

  float target_angle = wrap360(steer_angle_deg);
  float diff = shortestAngleError(target_angle, current_angle);
  if (fabsf(diff) > 90.0f) {
    drive_velocity_m_s = -drive_velocity_m_s;
    target_angle = wrap360(target_angle + 180.0f);
  }

  float omega_rad_s = drive_velocity_m_s / wheel_radius_m_;
  float omega_deg_s = omega_rad_s * (180.0f / kPi);
  drive_ctrl_.setTarget(omega_deg_s);
  steer_ctrl_.setTarget(target_angle);
}

void SwerveModule::setStateRaw(float drive_velocity_deg_s, float steer_angle_deg) {
  drive_ctrl_.setTarget(drive_velocity_deg_s);
  steer_ctrl_.setTarget(wrap360(steer_angle_deg));
}

void SwerveModule::stop() {
  if (drive_motor_ != nullptr) {
    drive_motor_->stop();
  }
  if (steer_motor_ != nullptr) {
    steer_motor_->stop();
  }
}

void SwerveModule::update() {
  drive_ctrl_.update();
  steer_ctrl_.update();
}

const SwerveModuleConfig& SwerveModule::config() const {
  return cfg_;
}

SmartMotorController& SwerveModule::driveController() {
  return drive_ctrl_;
}

SmartMotorController& SwerveModule::steerController() {
  return steer_ctrl_;
}

Encoder* SwerveModule::driveEncoder() {
  return drive_encoder_;
}

Encoder* SwerveModule::steerEncoder() {
  return steer_encoder_;
}

float SwerveModule::defaultWheelDiameter(float wheel_diameter_m) {
  return wheel_diameter_m > 0.0f ? wheel_diameter_m : (2.0f * 0.0254f);
}

SwerveDrive::SwerveDrive(SwerveModule** modules, size_t module_count, float vref)
    : modules_(modules),
      module_count_(module_count),
      vref_(vref),
      drive_velocity_m_s_(0.0f),
      steer_angle_deg_(0.0f) {}

void SwerveDrive::begin(const SwerveGains& gains) {
  for (size_t i = 0; i < module_count_; i++) {
    if (modules_[i] != nullptr) {
      modules_[i]->begin(gains);
    }
  }
}

void SwerveDrive::setState(float drive_velocity_m_s, float steer_angle_deg) {
  drive_velocity_m_s_ = drive_velocity_m_s;
  steer_angle_deg_ = steer_angle_deg;
  for (size_t i = 0; i < module_count_; i++) {
    if (modules_[i] != nullptr) {
      modules_[i]->setState(drive_velocity_m_s_, steer_angle_deg_);
    }
  }
}

void SwerveDrive::setModuleState(size_t index, float drive_velocity_m_s, float steer_angle_deg) {
  if (index < module_count_ && modules_[index] != nullptr) {
    modules_[index]->setState(drive_velocity_m_s, steer_angle_deg);
  }
}

void SwerveDrive::drive(float vx_m_s, float vy_m_s, float omega_rad_s) {
  drive_velocity_m_s_ = sqrtf((vx_m_s * vx_m_s) + (vy_m_s * vy_m_s));
  for (size_t i = 0; i < module_count_; i++) {
    SwerveModule* m = modules_[i];
    if (m == nullptr) {
      continue;
    }
    const SwerveModuleConfig& cfg = m->config();
    float wx = vx_m_s - (omega_rad_s * cfg.location_y_m);
    float wy = vy_m_s + (omega_rad_s * cfg.location_x_m);
    float speed = sqrtf((wx * wx) + (wy * wy));
    float angle = atan2f(wy, wx) * (180.0f / kPi);
    if (speed < 0.01f) {
      m->driveController().setTarget(0.0f);
    } else {
      m->setState(speed, angle);
    }
  }
}

void SwerveDrive::zeroSteerAll() {
  for (size_t i = 0; i < module_count_; i++) {
    if (modules_[i] != nullptr) {
      modules_[i]->zeroSteerHere();
    }
  }
}

void SwerveDrive::stop() {
  for (size_t i = 0; i < module_count_; i++) {
    if (modules_[i] != nullptr) {
      modules_[i]->stop();
    }
  }
}

void SwerveDrive::update() {
  for (size_t i = 0; i < module_count_; i++) {
    if (modules_[i] != nullptr) {
      modules_[i]->update();
    }
  }
}

SwerveModule* SwerveDrive::module(size_t index) {
  if (index >= module_count_) {
    return nullptr;
  }
  return modules_[index];
}

size_t SwerveDrive::moduleCount() const {
  return module_count_;
}

float SwerveDrive::vref() const {
  return vref_;
}

}  // namespace robot
