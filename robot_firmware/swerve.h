// SPDX-FileCopyrightText: 2026 Robot project contributors
//
// SPDX-License-Identifier: MIT

#ifndef ROBOT_SWERVE_H
#define ROBOT_SWERVE_H

#include <Arduino.h>

#include "dcmotor.h"
#include "encoder.h"
#include "smart_motor.h"

namespace robot {

struct GainGroup {
  float kp;
  float ki;
  float kd;
  float kv;
  float ks;
};

struct SwerveGains {
  GainGroup drive_vel;
  GainGroup steer_pos;
  GainGroup steer_vel;
};

struct SwerveModuleConfig {
  const char* name;
  float wheel_diameter_m;
  float location_x_m;
  float location_y_m;
  float drive_vel_sign;
  float drive_output_sign;
  float steer_vel_sign;
  float steer_output_sign;
  float steer_pos_vel_limit;
  bool has_steer_pos_vel_limit;
  bool steer_pos_feedforward;
  float steer_pos_deadband;
  float drive_static_ff;
  float steer_static_ff;
  float drive_voltage_slew;
  float steer_voltage_slew;
};

class SwerveModule {
 public:
  SwerveModule(const SwerveModuleConfig& config,
               DCMotor* drive_motor,
               DCMotor* steer_motor,
               Encoder* drive_encoder,
               Encoder* steer_encoder,
               float vref = 3.3f);

  bool begin(const SwerveGains& gains);
  void applyDefaultGains(const SwerveGains& gains);
  float zeroSteerHere();
  void setState(float drive_velocity_m_s, float steer_angle_deg);
  void setStateRaw(float drive_velocity_deg_s, float steer_angle_deg);
  void stop();
  void update();

  const SwerveModuleConfig& config() const;
  SmartMotorController& driveController();
  SmartMotorController& steerController();
  Encoder* driveEncoder();
  Encoder* steerEncoder();

 private:
  static float defaultWheelDiameter(float wheel_diameter_m);

  SwerveModuleConfig cfg_;
  float wheel_radius_m_;
  DCMotor* drive_motor_;
  DCMotor* steer_motor_;
  Encoder* drive_encoder_;
  Encoder* steer_encoder_;
  SmartMotorController drive_ctrl_;
  SmartMotorController steer_ctrl_;
};

class SwerveDrive {
 public:
  SwerveDrive(SwerveModule** modules, size_t module_count, float vref = 3.3f);

  void begin(const SwerveGains& gains);
  void setState(float drive_velocity_m_s, float steer_angle_deg);
  void setModuleState(size_t index, float drive_velocity_m_s, float steer_angle_deg);
  void drive(float vx_m_s, float vy_m_s, float omega_rad_s);
  void zeroSteerAll();
  void stop();
  void update();

  SwerveModule* module(size_t index);
  size_t moduleCount() const;
  float vref() const;

 private:
  SwerveModule** modules_;
  size_t module_count_;
  float vref_;
  float drive_velocity_m_s_;
  float steer_angle_deg_;
};

SwerveGains defaultSwerveGains();

}  // namespace robot

#endif  // ROBOT_SWERVE_H
