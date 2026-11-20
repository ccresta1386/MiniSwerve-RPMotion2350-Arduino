// SPDX-FileCopyrightText: 2026 Robot project contributors
//
// SPDX-License-Identifier: MIT

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>

#include "buzzer.h"
#include "dcmotor.h"
#include "encoder.h"
#include "startup_autotune.h"
#include "swerve.h"

namespace robot_config {

constexpr float kVref = 12.3f;
constexpr uint32_t kPwmFreqHz = 10000;
constexpr float kUpdatePeriodS = 0.005f;
constexpr float kMaxDriveVelMS = 2.0f;
constexpr float kMaxRotVelDegS = 180.0f;

constexpr uint8_t kButtonNextPin = 20;
constexpr uint8_t kButtonSelectPin = 21;
constexpr uint8_t kBuzzerPin = 22;
constexpr uint8_t kNeoPixelPin = 23;
constexpr uint8_t kNeoPixelCount = 2;

constexpr bool kControllerTestAtBoot = true;
constexpr float kControllerTestDurationS = 20.0f;

constexpr uint8_t kUsbHostDpPin = 24;
constexpr uint8_t kUsbHostDmPin = 25;
constexpr uint8_t kUsbHostInterface = 1;
constexpr uint8_t kUsbHostInEndpoint = 0x82;
constexpr uint8_t kUsbHostReportSize = 64;
constexpr uint16_t kUsbHostReadTimeoutMs = 20;
constexpr bool kUsbHostMapToDrive = true;
constexpr float kUsbHostDeadband = 0.08f;
constexpr uint8_t kUsbHostAxisCenter = 128;
constexpr uint8_t kUsbHostAxisSpan = 127;
constexpr uint8_t kUsbHostVxIndex = 1;
constexpr uint8_t kUsbHostVyIndex = 0;
constexpr uint8_t kUsbHostOmegaIndex = 2;
constexpr bool kUsbHostInvertVx = true;
constexpr bool kUsbHostInvertVy = false;
constexpr bool kUsbHostInvertOmega = false;
constexpr float kUsbHostCommandTimeoutS = 0.25f;

extern robot::SwerveGains global_gains;
extern robot::SwerveGains front_left_gains;
extern robot::SwerveGains back_right_gains;
extern robot::StartupAutotuneConfig startup_autotune_config;

extern robot::Buzzer buzzer;

extern robot::DCMotor front_left_drive_motor;
extern robot::DCMotor front_left_steer_motor;
extern robot::IncrementalEncoder front_left_drive_encoder;
extern robot::AbsoluteEncoder front_left_steer_encoder;
extern robot::SwerveModule front_left_module;

extern robot::DCMotor back_right_drive_motor;
extern robot::DCMotor back_right_steer_motor;
extern robot::IncrementalEncoder back_right_drive_encoder;
extern robot::AbsoluteEncoder back_right_steer_encoder;
extern robot::SwerveModule back_right_module;

extern robot::SwerveModule* swerve_modules[];
extern robot::SwerveDrive swerve_drive;

void beginRobotHardware();
void applyPerModuleGains();
bool saveSteerOffsets();
const __FlashStringHelper* lastSteerOffsetSaveStatus();
const __FlashStringHelper* lastSteerOffsetLoadStatus();

}  // namespace robot_config

#endif  // ROBOT_CONFIG_H
