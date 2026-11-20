// SPDX-FileCopyrightText: 2026 Robot project contributors
//
// SPDX-License-Identifier: MIT

#ifndef ROBOT_STARTUP_AUTOTUNE_H
#define ROBOT_STARTUP_AUTOTUNE_H

#include <Arduino.h>

#include "buzzer.h"
#include "swerve.h"

namespace robot {

struct StartupAutotuneAxisConfig {
  bool enabled;
  float step_voltage;
  float kick_voltage;
  float kick_time_s;
  float baseline_time_s;
  float measure_time_s;
  float sample_period_s;
  float min_delta_velocity;
  bool bidirectional_fallback;
  float pole_min;
  float pole_max;
  float kp_min;
  float kp_max;
  float ki_min;
  float ki_max;
  float steer_kp_scale;
};

struct StartupAutotuneConfig {
  bool enabled;
  StartupAutotuneAxisConfig drive;
  StartupAutotuneAxisConfig steer;
};

struct StartupAutotuneModuleResult {
  bool drive_ok;
  bool steer_ok;
  const char* drive_error;
  const char* steer_error;
  GainGroup drive_gains;
  float steer_pos_kp;
  float steer_kv;
};

struct StartupAutotuneSummary {
  bool ran;
  bool changed;
  const char* reason;
  size_t module_count;
};

StartupAutotuneConfig defaultStartupAutotuneConfig();

StartupAutotuneSummary runStartupAutotune(SwerveDrive& drive,
                                          const StartupAutotuneConfig& cfg,
                                          Buzzer* buzzer = nullptr,
                                          StartupAutotuneModuleResult* results = nullptr,
                                          size_t result_capacity = 0);

}  // namespace robot

#endif  // ROBOT_STARTUP_AUTOTUNE_H
