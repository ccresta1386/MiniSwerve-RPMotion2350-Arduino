// SPDX-FileCopyrightText: 2026 Robot project contributors
//
// SPDX-License-Identifier: MIT

#include "startup_autotune.h"

#include <math.h>

namespace robot {
namespace {

constexpr size_t kMaxSamples = 80;

struct VelocitySample {
  float t_s;
  float velocity;
};

struct PlantModel {
  bool ok;
  const char* error;
  float pole_a;
  float coeff_b;
  float gain_k;
  float sample_period_s;
  float time_constant_s;
  float baseline_velocity;
  float steady_velocity;
  float delta_velocity;
  float suggested_kv;
  float step_voltage;
};

float clampFloat(float value, float low, float high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

void sleepRemaining(uint32_t loop_start_us, float period_s) {
  uint32_t period_us = static_cast<uint32_t>(period_s * 1000000.0f);
  uint32_t elapsed = micros() - loop_start_us;
  if (elapsed < period_us) {
    delayMicroseconds(period_us - elapsed);
  }
}

void safeBeep(Buzzer* buzzer, uint16_t freq, uint16_t duration_ms) {
  if (buzzer != nullptr) {
    buzzer->beep(freq, duration_ms);
  }
}

float median(float* values, size_t count) {
  if (count == 0) {
    return NAN;
  }
  for (size_t i = 1; i < count; i++) {
    float key = values[i];
    size_t j = i;
    while (j > 0 && values[j - 1] > key) {
      values[j] = values[j - 1];
      j--;
    }
    values[j] = key;
  }
  size_t mid = count / 2;
  if ((count & 1u) != 0u) {
    return values[mid];
  }
  return 0.5f * (values[mid - 1] + values[mid]);
}

size_t collectVelocityStep(SmartMotorController& ctrl,
                           Encoder* encoder,
                           float step_voltage,
                           const StartupAutotuneAxisConfig& cfg,
                           VelocitySample* samples,
                           size_t sample_capacity,
                           float* baseline_out) {
  if (encoder == nullptr || samples == nullptr || sample_capacity == 0) {
    *baseline_out = 0.0f;
    return 0;
  }

  SmartMotorController::Mode original_mode = ctrl.mode();
  ctrl.setMode(SmartMotorController::MODE_VOLTAGE);
  ctrl.setTarget(0.0f);

  float baseline_accum = 0.0f;
  uint16_t baseline_count = 0;
  uint32_t baseline_start_us = micros();
  while ((micros() - baseline_start_us) < static_cast<uint32_t>(cfg.baseline_time_s * 1000000.0f)) {
    uint32_t loop_start = micros();
    ctrl.update();
    baseline_accum += encoder->getAngularVelocityDegS();
    baseline_count++;
    sleepRemaining(loop_start, cfg.sample_period_s);
  }
  *baseline_out = baseline_count > 0 ? baseline_accum / static_cast<float>(baseline_count) : 0.0f;

  float kick_voltage = fabsf(cfg.kick_voltage);
  if (cfg.kick_time_s > 0.0f && kick_voltage > fabsf(step_voltage)) {
    ctrl.setTarget(step_voltage >= 0.0f ? kick_voltage : -kick_voltage);
    uint32_t kick_start_us = micros();
    while ((micros() - kick_start_us) < static_cast<uint32_t>(cfg.kick_time_s * 1000000.0f)) {
      uint32_t loop_start = micros();
      ctrl.update();
      sleepRemaining(loop_start, cfg.sample_period_s);
    }
  }

  ctrl.setTarget(step_voltage);
  size_t sample_count = 0;
  uint32_t step_start_us = micros();
  while ((micros() - step_start_us) < static_cast<uint32_t>(cfg.measure_time_s * 1000000.0f)) {
    uint32_t loop_start = micros();
    ctrl.update();
    if (sample_count < sample_capacity) {
      samples[sample_count].t_s = static_cast<float>(micros() - step_start_us) / 1000000.0f;
      samples[sample_count].velocity = encoder->getAngularVelocityDegS();
      sample_count++;
    }
    sleepRemaining(loop_start, cfg.sample_period_s);
  }

  ctrl.setTarget(0.0f);
  uint32_t cool_start_us = micros();
  while ((micros() - cool_start_us) < 30000u) {
    ctrl.update();
    delay(1);
  }
  ctrl.setMode(original_mode);
  ctrl.setTarget(0.0f);
  return sample_count;
}

PlantModel estimateDiscreteFirstOrder(float baseline,
                                      const VelocitySample* samples,
                                      size_t sample_count,
                                      float step_voltage,
                                      float min_delta_velocity) {
  PlantModel model = {};
  model.ok = false;
  model.error = nullptr;
  model.step_voltage = step_voltage;
  if (sample_count < 6) {
    model.error = "insufficient samples";
    return model;
  }

  float dts[kMaxSamples];
  size_t dt_count = 0;
  for (size_t i = 1; i < sample_count && dt_count < kMaxSamples; i++) {
    float dt = samples[i].t_s - samples[i - 1].t_s;
    if (dt > 0.0001f) {
      dts[dt_count++] = dt;
    }
  }
  float ts = median(dts, dt_count);
  if (isnan(ts) || ts <= 0.0f) {
    model.error = "invalid sample period";
    return model;
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

  float delta = steady - baseline;
  if (fabsf(delta) < min_delta_velocity) {
    model.error = "response too small";
    model.steady_velocity = steady;
    model.delta_velocity = delta;
    return model;
  }

  float ratios[kMaxSamples];
  size_t ratio_count = 0;
  float min_err = fmaxf(1e-6f, fabsf(delta) * 0.08f);
  for (size_t i = 0; i + 1 < sample_count && ratio_count < kMaxSamples; i++) {
    float yk = samples[i].velocity - baseline;
    float yk1 = samples[i + 1].velocity - baseline;
    float err_k = delta - yk;
    float err_k1 = delta - yk1;
    if (fabsf(err_k) <= min_err) {
      continue;
    }
    float ratio = err_k1 / err_k;
    if (ratio > 0.02f && ratio < 1.05f) {
      ratios[ratio_count++] = ratio;
    }
  }

  float a = median(ratios, ratio_count);
  if (isnan(a)) {
    float threshold = baseline + (0.632f * delta);
    float t63 = NAN;
    for (size_t i = 0; i < sample_count; i++) {
      float vel = samples[i].velocity;
      if ((delta > 0.0f && vel >= threshold) || (delta < 0.0f && vel <= threshold)) {
        t63 = samples[i].t_s;
        break;
      }
    }
    if (isnan(t63)) {
      model.error = "could not estimate discrete pole";
      return model;
    }
    a = expf(-ts / fmaxf(t63, ts));
  }

  a = clampFloat(a, 0.05f, 0.995f);
  float k = delta / step_voltage;
  float b = (1.0f - a) * k;
  if (b <= 0.0f) {
    model.error = "non-positive plant gain";
    model.pole_a = a;
    model.gain_k = k;
    model.coeff_b = b;
    return model;
  }

  model.ok = true;
  model.pole_a = a;
  model.coeff_b = b;
  model.gain_k = k;
  model.sample_period_s = ts;
  model.time_constant_s = -ts / logf(a);
  model.baseline_velocity = baseline;
  model.steady_velocity = steady;
  model.delta_velocity = delta;
  model.suggested_kv = step_voltage / delta;
  return model;
}

PlantModel identifyVelocity(SmartMotorController& ctrl,
                            Encoder* encoder,
                            const StartupAutotuneAxisConfig& cfg,
                            float vref) {
  VelocitySample samples[kMaxSamples];
  float steps[2] = {
      clampFloat(fabsf(cfg.step_voltage), 0.0f, vref),
      -clampFloat(fabsf(cfg.step_voltage), 0.0f, vref),
  };
  uint8_t attempts = cfg.bidirectional_fallback ? 2 : 1;
  PlantModel best = {};
  best.ok = false;
  best.error = "no identification attempt";
  float best_delta = -1.0f;

  for (uint8_t i = 0; i < attempts; i++) {
    float baseline = 0.0f;
    size_t count = collectVelocityStep(ctrl, encoder, steps[i], cfg, samples, kMaxSamples, &baseline);
    PlantModel model = estimateDiscreteFirstOrder(
        baseline, samples, count, steps[i], cfg.min_delta_velocity);
    if (model.ok) {
      return model;
    }
    float delta_mag = fabsf(model.delta_velocity);
    if (delta_mag > best_delta) {
      best_delta = delta_mag;
      best = model;
    }
  }
  return best;
}

GainGroup designDrivePi(const PlantModel& model, const StartupAutotuneAxisConfig& cfg) {
  float a = model.pole_a;
  float b = model.coeff_b;
  float ts = model.sample_period_s;
  float p = clampFloat(a * a, cfg.pole_min, cfg.pole_max);

  GainGroup gains = {};
  gains.kp = (a + 1.0f - (2.0f * p)) / b;
  gains.ki = ((1.0f - p) * (1.0f - p)) / (b * ts);
  gains.kd = 0.0f;
  gains.kv = model.suggested_kv;
  gains.ks = 0.0f;
  gains.kp = clampFloat(gains.kp, cfg.kp_min, cfg.kp_max);
  gains.ki = clampFloat(gains.ki, cfg.ki_min, cfg.ki_max);
  return gains;
}

float designSteerPositionKp(const PlantModel& model, const StartupAutotuneAxisConfig& cfg) {
  float kp = cfg.steer_kp_scale *
             (((1.0f - model.pole_a) * (1.0f - model.pole_a)) /
              (4.0f * model.coeff_b * model.sample_period_s));
  return clampFloat(kp, cfg.kp_min, cfg.kp_max);
}

}  // namespace

StartupAutotuneConfig defaultStartupAutotuneConfig() {
  StartupAutotuneConfig cfg = {};
  cfg.enabled = false;
  cfg.drive = {
      true,   // enabled
      1.8f,   // step_voltage
      1.8f,   // kick_voltage
      0.12f,  // kick_time_s
      0.12f,  // baseline_time_s
      0.45f,  // measure_time_s
      0.01f,  // sample_period_s
      5.0f,   // min_delta_velocity
      true,   // bidirectional_fallback
      0.25f,  // pole_min
      0.92f,  // pole_max
      0.0f,   // kp_min
      0.3f,   // kp_max
      0.0f,   // ki_min
      12.0f,  // ki_max
      0.9f,   // steer_kp_scale
  };
  cfg.steer = {
      true,   // enabled
      0.8f,   // step_voltage
      0.8f,   // kick_voltage
      0.08f,  // kick_time_s
      0.12f,  // baseline_time_s
      0.35f,  // measure_time_s
      0.01f,  // sample_period_s
      1.0f,   // min_delta_velocity
      true,   // bidirectional_fallback
      0.25f,  // pole_min
      0.92f,  // pole_max
      0.01f,  // kp_min
      2.0f,   // kp_max
      0.0f,   // ki_min
      12.0f,  // ki_max
      0.9f,   // steer_kp_scale
  };
  return cfg;
}

StartupAutotuneSummary runStartupAutotune(SwerveDrive& drive,
                                          const StartupAutotuneConfig& cfg,
                                          Buzzer* buzzer,
                                          StartupAutotuneModuleResult* results,
                                          size_t result_capacity) {
  StartupAutotuneSummary summary = {};
  summary.ran = false;
  summary.changed = false;
  summary.reason = nullptr;
  summary.module_count = 0;

  if (!cfg.enabled) {
    summary.reason = "disabled";
    return summary;
  }
  if (drive.moduleCount() == 0) {
    summary.reason = "no modules";
    return summary;
  }
  if (!cfg.drive.enabled && !cfg.steer.enabled) {
    summary.reason = "both tuners disabled";
    return summary;
  }

  safeBeep(buzzer, 1500, 60);
  summary.ran = true;

  for (size_t i = 0; i < drive.moduleCount(); i++) {
    SwerveModule* module = drive.module(i);
    if (module == nullptr) {
      continue;
    }
    StartupAutotuneModuleResult module_result = {};
    module_result.drive_ok = false;
    module_result.steer_ok = false;
    module_result.drive_error = nullptr;
    module_result.steer_error = nullptr;

    if (cfg.drive.enabled) {
      PlantModel model = identifyVelocity(
          module->driveController(), module->driveEncoder(), cfg.drive, drive.vref());
      if (model.ok) {
        GainGroup gains = designDrivePi(model, cfg.drive);
        module->driveController().applyVelocityGains(gains.kp, gains.ki, gains.kd, gains.kv);
        module_result.drive_ok = true;
        module_result.drive_gains = gains;
        summary.changed = true;
      } else {
        module_result.drive_error = model.error;
      }
    }

    if (cfg.steer.enabled) {
      PlantModel model = identifyVelocity(
          module->steerController(), module->steerEncoder(), cfg.steer, drive.vref());
      if (model.ok) {
        float steer_kp = designSteerPositionKp(model, cfg.steer);
        module->steerController().pos_pid.kp = steer_kp;
        module->steerController().pos_pid.reset();
        module->steerController().applyVelocityGains(0.0f, 0.0f, 0.0f, model.suggested_kv);
        module_result.steer_ok = true;
        module_result.steer_pos_kp = steer_kp;
        module_result.steer_kv = model.suggested_kv;
        summary.changed = true;
      } else {
        module_result.steer_error = model.error;
      }
    }

    module->driveController().setTarget(0.0f);
    Encoder* steer_encoder = module->steerEncoder();
    module->steerController().setTarget(steer_encoder != nullptr ? steer_encoder->getAngleDeg() : 0.0f);

    if (results != nullptr && summary.module_count < result_capacity) {
      results[summary.module_count] = module_result;
    }
    summary.module_count++;
  }

  safeBeep(buzzer, summary.changed ? 1900 : 900, summary.changed ? 70 : 90);
  return summary;
}

}  // namespace robot
