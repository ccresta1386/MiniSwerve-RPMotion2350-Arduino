// SPDX-FileCopyrightText: 2026 Robot project contributors
//
// SPDX-License-Identifier: MIT

#include "robot_config.h"

#include <EEPROM.h>

namespace robot_config {
namespace {

const __FlashStringHelper* g_last_steer_offset_save_status = F("not-run");
const __FlashStringHelper* g_last_steer_offset_load_status = F("not-run");

constexpr uint32_t kPersistMagic = 0x53575256UL;  // "SWRV"
constexpr uint16_t kPersistVersion = 1;
constexpr int kEepromSize = 64;

struct PersistedSteerOffsets {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  float front_left_offset_deg;
  float back_right_offset_deg;
  uint32_t checksum;
};

float normalizeDeg(float value) {
  float out = fmodf(value, 360.0f);
  if (out < 0.0f) {
    out += 360.0f;
  }
  return out;
}

uint32_t checksumOffsets(PersistedSteerOffsets const& data) {
  PersistedSteerOffsets temp = data;
  temp.checksum = 0;

  uint32_t hash = 2166136261UL;
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&temp);
  for (size_t i = 0; i < sizeof(PersistedSteerOffsets); i++) {
    hash ^= bytes[i];
    hash *= 16777619UL;
  }
  return hash;
}

PersistedSteerOffsets makePersistedSteerOffsets() {
  PersistedSteerOffsets data = {};
  data.magic = kPersistMagic;
  data.version = kPersistVersion;
  data.size = sizeof(PersistedSteerOffsets);
  data.front_left_offset_deg = normalizeDeg(front_left_steer_encoder.offsetDeg());
  data.back_right_offset_deg = normalizeDeg(back_right_steer_encoder.offsetDeg());
  data.checksum = checksumOffsets(data);
  return data;
}

bool persistedSteerOffsetsValid(PersistedSteerOffsets const& data) {
  return data.magic == kPersistMagic &&
         data.version == kPersistVersion &&
         data.size == sizeof(PersistedSteerOffsets) &&
         data.checksum == checksumOffsets(data) &&
         data.front_left_offset_deg >= 0.0f &&
         data.front_left_offset_deg < 360.0f &&
         data.back_right_offset_deg >= 0.0f &&
         data.back_right_offset_deg < 360.0f;
}

bool loadSteerOffsetsFromEeprom() {
  EEPROM.begin(kEepromSize);
  PersistedSteerOffsets data = {};
  EEPROM.get(0, data);
  if (!persistedSteerOffsetsValid(data)) {
    g_last_steer_offset_load_status = F("no-valid-eeprom-offsets");
    return false;
  }
  front_left_steer_encoder.setOffsetDeg(data.front_left_offset_deg);
  back_right_steer_encoder.setOffsetDeg(data.back_right_offset_deg);
  g_last_steer_offset_load_status = F("loaded-eeprom");
  return true;
}

robot::SwerveModuleConfig frontLeftConfig() {
  robot::SwerveModuleConfig cfg = {};
  cfg.name = "front_left";
  cfg.wheel_diameter_m = 0.0508f;
  cfg.location_x_m = 0.25f;
  cfg.location_y_m = 0.25f;
  cfg.drive_vel_sign = 1.0f;
  cfg.drive_output_sign = 1.0f;
  cfg.steer_vel_sign = 1.0f;
  cfg.steer_output_sign = 1.0f;
  cfg.steer_pos_vel_limit = 90.0f;
  cfg.has_steer_pos_vel_limit = true;
  cfg.steer_pos_feedforward = false;
  cfg.steer_pos_deadband = 1.0f;
  cfg.drive_static_ff = 2.0f;
  cfg.steer_static_ff = 0.0f;
  cfg.drive_voltage_slew = 20.5f;
  cfg.steer_voltage_slew = 15.0f;
  return cfg;
}

robot::SwerveModuleConfig backRightConfig() {
  robot::SwerveModuleConfig cfg = {};
  cfg.name = "back_right";
  cfg.wheel_diameter_m = 0.0508f;
  cfg.location_x_m = -0.25f;
  cfg.location_y_m = -0.25f;
  cfg.drive_vel_sign = 1.0f;
  cfg.drive_output_sign = 1.0f;
  cfg.steer_vel_sign = 1.0f;
  cfg.steer_output_sign = 1.0f;
  cfg.steer_pos_vel_limit = 90.0f;
  cfg.has_steer_pos_vel_limit = true;
  cfg.steer_pos_feedforward = false;
  cfg.steer_pos_deadband = 2.0f;
  cfg.drive_static_ff = 2.0f;
  cfg.steer_static_ff = 0.0f;
  cfg.drive_voltage_slew = 20.5f;
  cfg.steer_voltage_slew = 15.0f;
  return cfg;
}

robot::StartupAutotuneConfig makeStartupAutotuneConfig() {
  robot::StartupAutotuneConfig cfg = robot::defaultStartupAutotuneConfig();
  cfg.enabled = true;

  cfg.drive.enabled = false;
  cfg.drive.step_voltage = 11.5f;
  cfg.drive.kick_voltage = 12.3f;
  cfg.drive.kick_time_s = 0.12f;
  cfg.drive.baseline_time_s = 0.1f;
  cfg.drive.measure_time_s = 0.25f;
  cfg.drive.sample_period_s = 0.01f;
  cfg.drive.min_delta_velocity = 1.0f;
  cfg.drive.bidirectional_fallback = true;
  cfg.drive.pole_min = 0.2f;
  cfg.drive.pole_max = 0.92f;
  cfg.drive.kp_min = 0.0f;
  cfg.drive.kp_max = 1.8f;
  cfg.drive.ki_min = 0.0f;
  cfg.drive.ki_max = 0.08f;

  cfg.steer.enabled = true;
  cfg.steer.step_voltage = 2.0f;
  cfg.steer.kick_voltage = 4.0f;
  cfg.steer.kick_time_s = 0.08f;
  cfg.steer.baseline_time_s = 0.25f;
  cfg.steer.measure_time_s = 0.5f;
  cfg.steer.sample_period_s = 0.01f;
  cfg.steer.min_delta_velocity = 1.0f;
  cfg.steer.bidirectional_fallback = true;
  cfg.steer.steer_kp_scale = 1.6f;
  cfg.steer.kp_min = 0.01f;
  cfg.steer.kp_max = 0.8f;
  return cfg;
}

robot::SwerveGains makeGlobalGains() {
  robot::SwerveGains gains = robot::defaultSwerveGains();
  gains.drive_vel = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  gains.steer_pos = {0.1f, 0.0f, 0.0f, 0.0f, 0.0f};
  return gains;
}

robot::SwerveGains makeFrontLeftGains() {
  robot::SwerveGains gains = makeGlobalGains();
  gains.drive_vel = {0.0001f, 0.0f, 0.0f, 0.001f, 0.65f};
  gains.steer_pos = {0.04f, 0.0f, 0.0f, 0.0f, 0.0f};
  return gains;
}

robot::SwerveGains makeBackRightGains() {
  robot::SwerveGains gains = makeGlobalGains();
  gains.drive_vel = {0.0001f, 0.0f, 0.0f, 0.001f, 0.70f};
  gains.steer_pos = {0.04f, 0.0f, 0.0f, 0.0f, 0.0f};
  return gains;
}

}  // namespace

robot::SwerveGains global_gains = makeGlobalGains();
robot::SwerveGains front_left_gains = makeFrontLeftGains();
robot::SwerveGains back_right_gains = makeBackRightGains();
robot::StartupAutotuneConfig startup_autotune_config = makeStartupAutotuneConfig();

robot::Buzzer buzzer(kBuzzerPin, 0.1f);

robot::DCMotor front_left_drive_motor(8, 9, kVref, kPwmFreqHz, false);
robot::DCMotor front_left_steer_motor(11, 10, kVref, kPwmFreqHz, false);
robot::IncrementalEncoder front_left_drive_encoder(0, 1, 72.0f, true, 0.0f, 0);
robot::AbsoluteEncoder front_left_steer_encoder(26, 65536.0f, false, 208.11f);
robot::SwerveModule front_left_module(
    frontLeftConfig(),
    &front_left_drive_motor,
    &front_left_steer_motor,
    &front_left_drive_encoder,
    &front_left_steer_encoder,
    kVref);

robot::DCMotor back_right_drive_motor(12, 13, kVref, kPwmFreqHz, false);
robot::DCMotor back_right_steer_motor(14, 15, kVref, kPwmFreqHz, false);
robot::IncrementalEncoder back_right_drive_encoder(2, 3, 72.0f, false, 0.0f, 1);
robot::AbsoluteEncoder back_right_steer_encoder(27, 65536.0f, false, 335.28f);
robot::SwerveModule back_right_module(
    backRightConfig(),
    &back_right_drive_motor,
    &back_right_steer_motor,
    &back_right_drive_encoder,
    &back_right_steer_encoder,
    kVref);

robot::SwerveModule* swerve_modules[] = {
    &front_left_module,
    &back_right_module,
};

robot::SwerveDrive swerve_drive(
    swerve_modules,
    sizeof(swerve_modules) / sizeof(swerve_modules[0]),
    kVref);

void beginRobotHardware() {
  loadSteerOffsetsFromEeprom();
  buzzer.begin();
  swerve_drive.begin(global_gains);
  applyPerModuleGains();
}

void applyPerModuleGains() {
  front_left_module.applyDefaultGains(front_left_gains);
  back_right_module.applyDefaultGains(back_right_gains);
}

bool saveSteerOffsets() {
  EEPROM.begin(kEepromSize);
  const PersistedSteerOffsets data = makePersistedSteerOffsets();
  EEPROM.put(0, data);
  if (!EEPROM.commit()) {
    g_last_steer_offset_save_status = F("eeprom-commit-failed");
    return false;
  }
  g_last_steer_offset_save_status = F("saved-eeprom");
  return true;
}

const __FlashStringHelper* lastSteerOffsetSaveStatus() {
  return g_last_steer_offset_save_status;
}

const __FlashStringHelper* lastSteerOffsetLoadStatus() {
  return g_last_steer_offset_load_status;
}

}  // namespace robot_config
