#include <Arduino.h>
#include <atomic>
#include <math.h>
#include <string.h>
#include "robot_config.h"
#include "usbh_helper.h"
#include "tusb.h"

// USB host HID probe for controller mapping.
// Designed for RP2040/RP2350 boards that expose USBHost + TinyUSB callbacks.

static const uint32_t SERIAL_BAUD = 115200;
static const uint16_t MAX_REPORT_LEN = 64;
static const uint32_t STATS_INTERVAL_MS = 8000;
static const uint16_t UNCHANGED_HEARTBEAT_N = 200;
static const bool IGNORE_IDX0_DELTA = true;
static const bool SKIP_MIRRORED_RID0 = true;
static const bool REPORT_ID_FILTER_ENABLED = false;
static const uint8_t REPORT_ID_FILTER = 0x04;
static const bool LOG_BUTTON_LIST_1HZ = true;
static const uint32_t BUTTON_LIST_LOG_INTERVAL_MS = 1000;
static const bool LOG_RAW_REPORTS = false;
static const bool LOG_DELTA_LINES = false;
static const bool LOG_COMPACT_STATE = false;
static const uint32_t COMPACT_LOG_MIN_INTERVAL_MS = 40;
static const uint32_t COMPACT_IDLE_HEARTBEAT_MS = 1500;
static const int16_t AXIS_MIN_DELTA_FOR_LOG = 3;
static const bool LOG_BUTTON_BYTES = false;
static const uint8_t BUTTON_BYTE_FIRST = 23;
static const uint8_t BUTTON_BYTE_LAST = 26;
static const bool LOG_BUTTON_EVENTS = false;
static const uint8_t BUTTON_EVENT_BYTE_FIRST = 13;
static const uint8_t BUTTON_EVENT_BYTE_LAST = 31;
static const uint8_t BUTTON_EVENT_STABLE_REPORTS = 1;
static const uint32_t BUTTON_EVENT_MIN_INTERVAL_MS = 0;
static const bool BUTTON_EVENT_IGNORE_FULLBYTE_FLIPS = false;
static const uint8_t BUTTON_EVENT_MAX_TOGGLED_BITS = 2;
static const bool BUTTON_EVENT_SKIP_AXIS_BYTES = true;
static const bool BUTTON_EVENT_SKIP_NOISY_RANGE = false;
static const uint8_t BUTTON_EVENT_NOISY_FIRST = 15;
static const uint8_t BUTTON_EVENT_NOISY_LAST = 22;
static const uint8_t HOST_RHPORT = 1;
static const uint32_t HOST_TASK_TIMEOUT_MS = 2;
static const uint32_t HOST_REPORT_REQUEUE_INTERVAL_MS = 100;
static const uint32_t HOST_MOUNT_SETTLE_MS = 300;
static const uint32_t HOST_NO_ACTIVE_REPORT_LOG_MS = 3000;
static const uint8_t MAX_HID_INSTANCES_TRACKED = 8;
static const uint8_t DRIVE_ENABLE_BUTTON_BYTE = 6;
static const uint8_t DRIVE_ENABLE_BUTTON_MASK = 0x02;  // RB
static const uint8_t DRIVE_STOP_BUTTON_BYTE = 5;
static const uint8_t DRIVE_STOP_BUTTON_MASK = 0x10;    // A
static const uint8_t DRIVE_HORN_BUTTON_BYTE = 5;
static const uint8_t DRIVE_HORN_BUTTON_MASK = 0x20;    // B
static const uint8_t DRIVE_ZERO_BUTTON_BYTE = 5;
static const uint8_t DRIVE_ZERO_BUTTON_MASK = 0x80;    // Y
static const uint32_t DRIVE_STATUS_INTERVAL_MS = 1000;
static const bool LOG_STEER_ENCODER_1HZ = true;
static const uint32_t STEER_ENCODER_LOG_INTERVAL_MS = 1000;
static const uint16_t STARTUP_AUTOTUNE_HOLD_MS = 350;
static const uint32_t USB_DISCONNECT_ALARM_STEP_MS = 50;
static const uint8_t USB_DISCONNECT_ALARM_PEAK_STEP = 8;
static const uint16_t USB_DISCONNECT_ALARM_LOW_HZ = 700;
static const uint16_t USB_DISCONNECT_ALARM_PITCH_STEP_HZ = 150;

static volatile bool g_device_mounted = false;
static volatile bool g_host_started = false;
static volatile uint8_t g_dev_addr = 0;
static volatile uint8_t g_mounted_instances = 0;
static volatile uint8_t g_active_instance = 0;
static volatile bool g_have_active_instance = false;
static volatile uint16_t g_vid = 0;
static volatile uint16_t g_pid = 0;
static volatile uint32_t g_last_mount_ms = 0;
static std::atomic<bool> g_usb_disconnect_pending{false};

static uint8_t g_last_report[MAX_REPORT_LEN];
static uint8_t g_min_byte[MAX_REPORT_LEN];
static uint8_t g_max_byte[MAX_REPORT_LEN];
static uint8_t g_or_mask[MAX_REPORT_LEN];
static uint8_t g_and_mask[MAX_REPORT_LEN];
static uint16_t g_last_len = 0;
static bool g_have_last_report = false;
static uint32_t g_report_count = 0;
static uint32_t g_change_count = 0;
static uint32_t g_unchanged_count = 0;
static uint32_t g_filtered_report_count = 0;
static volatile uint32_t g_last_valid_report_ms = 0;
static uint32_t g_last_no_report_log_ms = 0;
static uint32_t g_last_stats_ms = 0;
static volatile uint32_t g_last_wait_log_ms = 0;
static uint32_t g_last_button_list_log_ms = 0;
static uint32_t g_last_report_requeue_ms = 0;
static uint32_t g_last_compact_log_ms = 0;
static bool g_have_last_compact_axes = false;
static bool g_have_last_compact_buttons = false;

struct AxisState {
  int16_t lx;
  int16_t ly;
  int16_t rx;
  int16_t ry;
};

struct SharedGamepadCommand {
  volatile int16_t lx;
  volatile int16_t ly;
  volatile int16_t rx;
  volatile int16_t ry;
  volatile bool valid;
  volatile bool drive_enabled;
  volatile bool stop_requested;
  volatile bool horn_requested;
  volatile uint32_t zero_request_count;
  volatile uint32_t updated_ms;
};

static AxisState g_last_compact_axes = {0, 0, 0, 0};
static SharedGamepadCommand g_gamepad_cmd = {0, 0, 0, 0, false, false, false, false, 0, 0};
static uint8_t g_last_compact_buttons[(BUTTON_BYTE_LAST - BUTTON_BYTE_FIRST) + 1];
static const uint8_t BUTTON_EVENT_BYTE_COUNT = (BUTTON_EVENT_BYTE_LAST - BUTTON_EVENT_BYTE_FIRST) + 1;
static uint8_t g_button_candidate[BUTTON_EVENT_BYTE_COUNT];
static uint8_t g_button_candidate_count[BUTTON_EVENT_BYTE_COUNT];
static uint8_t g_button_stable[BUTTON_EVENT_BYTE_COUNT];
static bool g_button_have_stable[BUTTON_EVENT_BYTE_COUNT];
static uint32_t g_last_button_event_ms = 0;
static uint32_t g_last_drive_control_us = 0;
static uint32_t g_last_drive_status_ms = 0;
static bool g_drive_output_active = false;
static bool g_robot_hardware_started = false;
static bool g_last_zero_button_pressed = false;
static bool g_horn_active = false;
static bool g_disconnect_alarm_active = false;
static uint32_t g_handled_zero_request_count = 0;
static uint32_t g_last_steer_encoder_log_ms = 0;

static void reset_report_tracking();

static void print_hex_byte(uint8_t v) {
  if (v < 16) {
    Serial.print('0');
  }
  Serial.print(v, HEX);
}

static uint8_t hid_instance_mask(uint8_t instance) {
  if (instance >= MAX_HID_INSTANCES_TRACKED) {
    return 0;
  }
  return (uint8_t)(1u << instance);
}

static bool button_event_byte_enabled(uint8_t idx) {
  if (idx < BUTTON_EVENT_BYTE_FIRST || idx > BUTTON_EVENT_BYTE_LAST) {
    return false;
  }
  if (BUTTON_EVENT_SKIP_AXIS_BYTES && idx >= 9 && idx <= 12) {
    return false;
  }
  if (BUTTON_EVENT_SKIP_NOISY_RANGE &&
      idx >= BUTTON_EVENT_NOISY_FIRST && idx <= BUTTON_EVENT_NOISY_LAST) {
    return false;
  }
  return true;
}

static void print_bit_indices(uint8_t mask) {
  bool any = false;
  for (uint8_t bit = 0; bit < 8; bit++) {
    if ((mask & (1u << bit)) == 0) {
      continue;
    }
    if (any) {
      Serial.print(',');
    }
    Serial.print(bit);
    any = true;
  }
  if (!any) {
    Serial.print('-');
  }
}

static uint8_t popcount8(uint8_t v) {
  uint8_t c = 0;
  while (v) {
    c += (uint8_t)(v & 1u);
    v >>= 1u;
  }
  return c;
}

static void log_button_event(uint8_t rid, uint8_t byte_idx, uint8_t old_v, uint8_t new_v) {
  uint8_t pressed = (uint8_t)(~old_v & new_v);
  uint8_t released = (uint8_t)(old_v & ~new_v);

  Serial.print(F("BUTTON_EVENT rid=0x"));
  print_hex_byte(rid);
  Serial.print(F(" b"));
  Serial.print(byte_idx);
  Serial.print(F(" 0x"));
  print_hex_byte(old_v);
  Serial.print(F("->0x"));
  print_hex_byte(new_v);
  Serial.print(F(" +("));
  print_bit_indices(pressed);
  Serial.print(F(") -("));
  print_bit_indices(released);
  Serial.println(')');
}

static void process_button_events(uint8_t const* report, uint16_t len, uint32_t now) {
  if (!LOG_BUTTON_EVENTS || len == 0) {
    return;
  }

  uint8_t first = BUTTON_EVENT_BYTE_FIRST;
  uint8_t last = BUTTON_EVENT_BYTE_LAST;
  if (first >= len) {
    return;
  }
  if (last >= len) {
    last = (uint8_t)(len - 1);
  }

  for (uint8_t b = first; b <= last; b++) {
    if (!button_event_byte_enabled(b)) {
      continue;
    }
    uint8_t slot = (uint8_t)(b - BUTTON_EVENT_BYTE_FIRST);
    uint8_t v = report[b];

    if (g_button_candidate[slot] == v) {
      if (g_button_candidate_count[slot] < 255) {
        g_button_candidate_count[slot]++;
      }
    } else {
      g_button_candidate[slot] = v;
      g_button_candidate_count[slot] = 1;
    }

    if (g_button_candidate_count[slot] < BUTTON_EVENT_STABLE_REPORTS) {
      continue;
    }

    if (!g_button_have_stable[slot]) {
      g_button_stable[slot] = g_button_candidate[slot];
      g_button_have_stable[slot] = true;
      continue;
    }

    if (g_button_stable[slot] == g_button_candidate[slot]) {
      continue;
    }

    if ((now - g_last_button_event_ms) < BUTTON_EVENT_MIN_INTERVAL_MS) {
      g_button_stable[slot] = g_button_candidate[slot];
      continue;
    }

    if (BUTTON_EVENT_IGNORE_FULLBYTE_FLIPS &&
        ((g_button_stable[slot] == 0x00 && g_button_candidate[slot] == 0xFF) ||
         (g_button_stable[slot] == 0xFF && g_button_candidate[slot] == 0x00))) {
      g_button_stable[slot] = g_button_candidate[slot];
      continue;
    }

    uint8_t toggled = (uint8_t)(g_button_stable[slot] ^ g_button_candidate[slot]);
    if (BUTTON_EVENT_MAX_TOGGLED_BITS > 0 &&
        popcount8(toggled) > BUTTON_EVENT_MAX_TOGGLED_BITS) {
      g_button_stable[slot] = g_button_candidate[slot];
      continue;
    }

    uint8_t rid = (len > 0) ? report[0] : 0;
    log_button_event(rid, b, g_button_stable[slot], g_button_candidate[slot]);
    g_button_stable[slot] = g_button_candidate[slot];
    g_last_button_event_ms = now;
  }
}

static void reset_report_tracking() {
  memset(g_last_report, 0, sizeof(g_last_report));
  memset(g_min_byte, 0xFF, sizeof(g_min_byte));
  memset(g_max_byte, 0x00, sizeof(g_max_byte));
  memset(g_or_mask, 0x00, sizeof(g_or_mask));
  memset(g_and_mask, 0xFF, sizeof(g_and_mask));
  memset(g_last_compact_buttons, 0, sizeof(g_last_compact_buttons));
  memset(g_button_candidate, 0, sizeof(g_button_candidate));
  memset(g_button_candidate_count, 0, sizeof(g_button_candidate_count));
  memset(g_button_stable, 0, sizeof(g_button_stable));
  memset(g_button_have_stable, 0, sizeof(g_button_have_stable));
  g_last_len = 0;
  g_have_last_report = false;
  g_report_count = 0;
  g_change_count = 0;
  g_unchanged_count = 0;
  g_filtered_report_count = 0;
  g_have_last_compact_axes = false;
  g_have_last_compact_buttons = false;
  g_last_report_requeue_ms = 0;
  g_last_compact_axes.lx = 0;
  g_last_compact_axes.ly = 0;
  g_last_compact_axes.rx = 0;
  g_last_compact_axes.ry = 0;
  g_last_button_list_log_ms = 0;
  g_last_compact_log_ms = millis();
  g_last_button_event_ms = millis();
  g_last_stats_ms = millis();
  g_gamepad_cmd.valid = false;
  g_gamepad_cmd.drive_enabled = false;
  g_gamepad_cmd.stop_requested = false;
  g_gamepad_cmd.horn_requested = false;
  g_gamepad_cmd.updated_ms = 0;
  g_last_zero_button_pressed = false;
}

static void log_report_line(uint8_t const* report, uint16_t len) {
  Serial.print(F("REPORT #"));
  Serial.print(g_report_count);
  Serial.print(F(" len="));
  Serial.print(len);
  if (len > 0) {
    Serial.print(F(" rid=0x"));
    print_hex_byte(report[0]);
  }
  Serial.print(F(" data="));
  for (uint16_t i = 0; i < len; i++) {
    print_hex_byte(report[i]);
    if (i + 1 < len) {
      Serial.print(' ');
    }
  }
  Serial.println();
}

static bool is_valid_gamepad_report(uint8_t const* report, uint16_t len) {
  return len >= 13 &&
         report[0] == 0x04 &&
         report[1] == 0x1B &&
         report[2] == 0x74 &&
         report[3] == 0x06 &&
         report[4] == 0x01;
}

static void print_pressed_button(bool pressed, const __FlashStringHelper* name, bool* any) {
  if (!pressed) {
    return;
  }
  if (*any) {
    Serial.print(' ');
  }
  Serial.print(name);
  *any = true;
}

static void log_button_list(uint8_t const* report, uint16_t len) {
  Serial.print(F("BUTTONS "));

  if (!is_valid_gamepad_report(report, len)) {
    Serial.println(F("invalid-report"));
    return;
  }

  bool any = false;
  print_pressed_button((report[5] & 0x10) != 0, F("A"), &any);
  print_pressed_button((report[5] & 0x20) != 0, F("B"), &any);
  print_pressed_button((report[5] & 0x40) != 0, F("X"), &any);
  print_pressed_button((report[5] & 0x80) != 0, F("Y"), &any);
  print_pressed_button((report[6] & 0x01) != 0, F("LB"), &any);
  print_pressed_button((report[6] & 0x02) != 0, F("RB"), &any);

  if (!any) {
    Serial.print(F("none"));
  }
  Serial.println();
}

static void log_deltas(uint8_t const* report, uint16_t len) {
  bool any = false;
  uint16_t common_len = len < g_last_len ? len : g_last_len;
  uint16_t start = IGNORE_IDX0_DELTA ? 1 : 0;
  for (uint16_t i = start; i < common_len; i++) {
    if (report[i] == g_last_report[i]) {
      continue;
    }
    if (!any) {
      Serial.print(F("DELTA "));
      any = true;
    }
    Serial.print(F("["));
    Serial.print(i);
    Serial.print(F(":"));
    print_hex_byte(g_last_report[i]);
    Serial.print(F("->"));
    print_hex_byte(report[i]);
    Serial.print(F("] "));
  }
  if (len != g_last_len) {
    if (!any) {
      Serial.print(F("DELTA "));
      any = true;
    }
    Serial.print(F("[len:"));
    Serial.print(g_last_len);
    Serial.print(F("->"));
    Serial.print(len);
    Serial.print(F("] "));
  }
  if (any) {
    Serial.println();
  }
}

static bool decode_axes(uint8_t const* report, uint16_t len, AxisState* axes) {
  if (len < 13 || axes == nullptr) {
    return false;
  }
  axes->lx = (int16_t)report[9] - 128;
  axes->ly = 128 - (int16_t)report[10];
  axes->rx = (int16_t)report[11] - 128;
  axes->ry = 128 - (int16_t)report[12];
  return true;
}

static void update_drive_command_from_report(uint8_t const* report, uint16_t len, uint32_t now) {
  AxisState axes;
  if (!robot_config::kUsbHostMapToDrive || !is_valid_gamepad_report(report, len) ||
      !decode_axes(report, len, &axes)) {
    return;
  }

  bool enable_pressed = len > DRIVE_ENABLE_BUTTON_BYTE &&
                        ((report[DRIVE_ENABLE_BUTTON_BYTE] & DRIVE_ENABLE_BUTTON_MASK) != 0);
  bool stop_pressed = len > DRIVE_STOP_BUTTON_BYTE &&
                      ((report[DRIVE_STOP_BUTTON_BYTE] & DRIVE_STOP_BUTTON_MASK) != 0);
  bool horn_pressed = len > DRIVE_HORN_BUTTON_BYTE &&
                      ((report[DRIVE_HORN_BUTTON_BYTE] & DRIVE_HORN_BUTTON_MASK) != 0);
  bool zero_pressed = len > DRIVE_ZERO_BUTTON_BYTE &&
                      ((report[DRIVE_ZERO_BUTTON_BYTE] & DRIVE_ZERO_BUTTON_MASK) != 0);

  if (zero_pressed && !g_last_zero_button_pressed) {
    g_gamepad_cmd.zero_request_count++;
  }
  g_last_zero_button_pressed = zero_pressed;

  g_gamepad_cmd.lx = axes.lx;
  g_gamepad_cmd.ly = axes.ly;
  g_gamepad_cmd.rx = axes.rx;
  g_gamepad_cmd.ry = axes.ry;
  g_gamepad_cmd.drive_enabled = enable_pressed;
  g_gamepad_cmd.stop_requested = stop_pressed;
  g_gamepad_cmd.horn_requested = horn_pressed;
  g_gamepad_cmd.valid = true;
  g_gamepad_cmd.updated_ms = now;
}

static int16_t axis_by_index(AxisState const& axes, uint8_t index) {
  switch (index) {
    case 0: return axes.lx;
    case 1: return axes.ly;
    case 2: return axes.rx;
    case 3: return axes.ry;
    default: return 0;
  }
}

static float normalize_axis(int16_t value, bool inverted) {
  float scaled = (float)value / (float)robot_config::kUsbHostAxisSpan;
  if (scaled > 1.0f) {
    scaled = 1.0f;
  } else if (scaled < -1.0f) {
    scaled = -1.0f;
  }
  if (inverted) {
    scaled = -scaled;
  }
  if (fabsf(scaled) < robot_config::kUsbHostDeadband) {
    return 0.0f;
  }
  return scaled;
}

static void stop_drive_outputs() {
  if (!g_robot_hardware_started) {
    return;
  }
  robot_config::swerve_drive.stop();
  if (g_horn_active) {
    robot_config::buzzer.off();
    g_horn_active = false;
  }
  g_drive_output_active = false;
}

static void set_horn_output(bool active) {
  if (!g_robot_hardware_started || g_disconnect_alarm_active || active == g_horn_active) {
    return;
  }
  if (active) {
    robot_config::buzzer.on(1200);
  } else {
    robot_config::buzzer.off();
  }
  g_horn_active = active;
}

static void service_disconnect_alarm(uint32_t now_ms) {
  static uint32_t started_ms = 0;
  static uint16_t last_frequency_hz = 0;

  if (g_usb_disconnect_pending.exchange(false, std::memory_order_relaxed)) {
    set_horn_output(false);
    g_disconnect_alarm_active = true;
    started_ms = now_ms;
  }
  if (!g_disconnect_alarm_active) {
    return;
  }

  const uint32_t step = (now_ms - started_ms) / USB_DISCONNECT_ALARM_STEP_MS;
  const uint32_t last_step = 2 * USB_DISCONNECT_ALARM_PEAK_STEP;
  if (step > last_step) {
    robot_config::buzzer.off();
    g_disconnect_alarm_active = false;
    last_frequency_hz = 0;
    return;
  }

  const uint32_t pitch_step = step <= USB_DISCONNECT_ALARM_PEAK_STEP ? step : last_step - step;
  const uint16_t frequency_hz = USB_DISCONNECT_ALARM_LOW_HZ + pitch_step * USB_DISCONNECT_ALARM_PITCH_STEP_HZ;
  if (frequency_hz != last_frequency_hz) {
    robot_config::buzzer.on(frequency_hz);
    last_frequency_hz = frequency_hz;
  }
}

static void handle_zero_request(uint32_t request_count) {
  if (request_count == g_handled_zero_request_count || request_count == 0) {
    return;
  }
  g_handled_zero_request_count = request_count;

  stop_drive_outputs();
  float front_left_offset = robot_config::front_left_module.zeroSteerHere();
  float back_right_offset = robot_config::back_right_module.zeroSteerHere();
  bool saved = robot_config::saveSteerOffsets();

  Serial.print(F("STEER_ZERO front_left="));
  Serial.print(front_left_offset, 4);
  Serial.print(F(" back_right="));
  Serial.print(back_right_offset, 4);
  Serial.print(F(" eeprom="));
  Serial.print(saved ? F("saved") : F("save-failed"));
  Serial.print(F(" reason="));
  Serial.println(robot_config::lastSteerOffsetSaveStatus());
  if (!saved) {
    Serial.print(F("STEER_ZERO_CONFIG front_left_offset_deg="));
    Serial.print(front_left_offset, 4);
    Serial.print(F(" back_right_offset_deg="));
    Serial.println(back_right_offset, 4);
    Serial.println(F("STEER_ZERO_CONFIG update robot_config.cpp AbsoluteEncoder constructor offsets with these values."));
  }
}

static void log_steer_encoder_debug(uint32_t now_ms) {
  if (!LOG_STEER_ENCODER_1HZ ||
      (now_ms - g_last_steer_encoder_log_ms) < STEER_ENCODER_LOG_INTERVAL_MS) {
    return;
  }
  g_last_steer_encoder_log_ms = now_ms;

  Serial.print(F("STEER_ENCODER fl_raw="));
  Serial.print(robot_config::front_left_steer_encoder.readRaw());
  Serial.print(F("/"));
  Serial.print(robot_config::front_left_steer_encoder.adcMax());
  Serial.print(F(" fl_angle="));
  Serial.print(robot_config::front_left_steer_encoder.getAngleDeg(), 2);
  Serial.print(F(" br_raw="));
  Serial.print(robot_config::back_right_steer_encoder.readRaw());
  Serial.print(F("/"));
  Serial.print(robot_config::back_right_steer_encoder.adcMax());
  Serial.print(F(" br_angle="));
  Serial.println(robot_config::back_right_steer_encoder.getAngleDeg(), 2);
}

static bool startup_autotune_button_held() {
  pinMode(robot_config::kButtonSelectPin, INPUT_PULLUP);
  uint32_t start_ms = millis();
  while ((millis() - start_ms) < STARTUP_AUTOTUNE_HOLD_MS) {
    if (digitalRead(robot_config::kButtonSelectPin) != LOW) {
      return false;
    }
    delay(10);
  }
  return true;
}

static void maybe_run_startup_autotune() {
  if (!startup_autotune_button_held()) {
    Serial.println(F("Startup autotune skipped (GP21 not held)."));
    return;
  }

  Serial.println(F("Startup autotune requested by GP21 hold."));
  stop_drive_outputs();
  robot::StartupAutotuneModuleResult results[4];
  robot::StartupAutotuneSummary summary = robot::runStartupAutotune(
      robot_config::swerve_drive,
      robot_config::startup_autotune_config,
      &robot_config::buzzer,
      results,
      sizeof(results) / sizeof(results[0]));

  Serial.print(F("AUTOTUNE ran="));
  Serial.print(summary.ran ? F("yes") : F("no"));
  Serial.print(F(" changed="));
  Serial.print(summary.changed ? F("yes") : F("no"));
  Serial.print(F(" modules="));
  Serial.print(summary.module_count);
  if (summary.reason != nullptr) {
    Serial.print(F(" reason="));
    Serial.print(summary.reason);
  }
  Serial.println();

  for (size_t i = 0; i < summary.module_count && i < (sizeof(results) / sizeof(results[0])); i++) {
    Serial.print(F("  module["));
    Serial.print(i);
    Serial.print(F("] drive="));
    Serial.print(results[i].drive_ok ? F("ok") : F("skip"));
    if (results[i].drive_error != nullptr) {
      Serial.print(F("("));
      Serial.print(results[i].drive_error);
      Serial.print(F(")"));
    }
    Serial.print(F(" steer="));
    Serial.print(results[i].steer_ok ? F("ok") : F("skip"));
    if (results[i].steer_error != nullptr) {
      Serial.print(F("("));
      Serial.print(results[i].steer_error);
      Serial.print(F(")"));
    }
    Serial.println();
  }

  stop_drive_outputs();
}

static void service_swerve_drive(uint32_t now_ms) {
  if (!g_robot_hardware_started) {
    return;
  }

  uint32_t now_us = micros();
  uint32_t period_us = (uint32_t)(robot_config::kUpdatePeriodS * 1000000.0f);
  if (period_us == 0) {
    period_us = 1000;
  }
  if (g_last_drive_control_us != 0 && (now_us - g_last_drive_control_us) < period_us) {
    return;
  }
  g_last_drive_control_us = now_us;

  AxisState axes;
  axes.lx = g_gamepad_cmd.lx;
  axes.ly = g_gamepad_cmd.ly;
  axes.rx = g_gamepad_cmd.rx;
  axes.ry = g_gamepad_cmd.ry;
  bool command_valid = g_gamepad_cmd.valid;
  bool drive_enabled = g_gamepad_cmd.drive_enabled;
  bool stop_requested = g_gamepad_cmd.stop_requested;
  bool horn_requested = g_gamepad_cmd.horn_requested;
  uint32_t zero_request_count = g_gamepad_cmd.zero_request_count;
  uint32_t command_age_ms = now_ms - g_gamepad_cmd.updated_ms;
  bool command_fresh = command_age_ms <= (uint32_t)(robot_config::kUsbHostCommandTimeoutS * 1000.0f);
  bool host_ok = g_host_started && g_device_mounted && g_have_active_instance;

  if (host_ok && command_valid && command_fresh) {
    set_horn_output(horn_requested);
    handle_zero_request(zero_request_count);
  } else {
    set_horn_output(false);
  }

  if (!robot_config::kUsbHostMapToDrive || !host_ok || !command_valid || !command_fresh || stop_requested) {
    stop_drive_outputs();
    return;
  }

  if (!drive_enabled) {
    stop_drive_outputs();
    if ((now_ms - g_last_drive_status_ms) >= DRIVE_STATUS_INTERVAL_MS) {
      Serial.println(F("DRIVE disabled-hold-rb"));
      g_last_drive_status_ms = now_ms;
    }
    return;
  }

  float vx = normalize_axis(axis_by_index(axes, robot_config::kUsbHostVxIndex),
                            robot_config::kUsbHostInvertVx) *
             robot_config::kMaxDriveVelMS;
  float vy = normalize_axis(axis_by_index(axes, robot_config::kUsbHostVyIndex),
                            robot_config::kUsbHostInvertVy) *
             robot_config::kMaxDriveVelMS;
  float omega_deg_s = normalize_axis(axis_by_index(axes, robot_config::kUsbHostOmegaIndex),
                                     robot_config::kUsbHostInvertOmega) *
                      robot_config::kMaxRotVelDegS;
  float omega_rad_s = omega_deg_s * 0.017453292519943295f;

  robot_config::swerve_drive.drive(vx, vy, omega_rad_s);
  robot_config::swerve_drive.update();
  g_drive_output_active = drive_enabled && (vx != 0.0f || vy != 0.0f || omega_rad_s != 0.0f);

  if ((now_ms - g_last_drive_status_ms) >= DRIVE_STATUS_INTERVAL_MS) {
    Serial.print(F("DRIVE "));
    if (!drive_enabled) {
      Serial.print(F("disabled-hold-rb"));
    } else {
      Serial.print(F("vx="));
      Serial.print(vx, 3);
      Serial.print(F(" vy="));
      Serial.print(vy, 3);
      Serial.print(F(" omega="));
      Serial.print(omega_rad_s, 3);
    }
    Serial.println();
    g_last_drive_status_ms = now_ms;
  }
}

static int16_t abs_i16(int16_t v) {
  return (v >= 0) ? v : (int16_t)-v;
}

static bool range_changed(uint8_t const* report, uint16_t len, uint8_t first, uint8_t last) {
  if (!g_have_last_report || len != g_last_len || first >= len) {
    return true;
  }
  uint8_t capped_last = last;
  if (capped_last >= len) {
    capped_last = (uint8_t)(len - 1);
  }
  if (first > capped_last) {
    return false;
  }
  for (uint8_t i = first; i <= capped_last; i++) {
    if (report[i] != g_last_report[i]) {
      return true;
    }
  }
  return false;
}

static bool should_log_compact_state(uint8_t const* report, uint16_t len, uint32_t now, AxisState* axes_out) {
  AxisState axes;
  if (!decode_axes(report, len, &axes)) {
    return false;
  }
  if (axes_out != nullptr) {
    *axes_out = axes;
  }

  bool buttons_changed = false;
  if (LOG_BUTTON_BYTES) {
    if (!g_have_last_compact_buttons || len <= BUTTON_BYTE_FIRST) {
      buttons_changed = true;
    } else {
      uint8_t btn_last = BUTTON_BYTE_LAST;
      if (btn_last >= len) {
        btn_last = (uint8_t)(len - 1);
      }
      uint8_t idx = 0;
      for (uint8_t b = BUTTON_BYTE_FIRST; b <= btn_last; b++, idx++) {
        if (report[b] != g_last_compact_buttons[idx]) {
          buttons_changed = true;
          break;
        }
      }
    }
  }

  bool first_log = !g_have_last_compact_axes;
  if (first_log || buttons_changed) {
    return true;
  }

  bool axis_delta_big = (abs_i16((int16_t)(axes.lx - g_last_compact_axes.lx)) >= AXIS_MIN_DELTA_FOR_LOG) ||
                        (abs_i16((int16_t)(axes.ly - g_last_compact_axes.ly)) >= AXIS_MIN_DELTA_FOR_LOG) ||
                        (abs_i16((int16_t)(axes.rx - g_last_compact_axes.rx)) >= AXIS_MIN_DELTA_FOR_LOG) ||
                        (abs_i16((int16_t)(axes.ry - g_last_compact_axes.ry)) >= AXIS_MIN_DELTA_FOR_LOG);

  if (axis_delta_big && (now - g_last_compact_log_ms) >= COMPACT_LOG_MIN_INTERVAL_MS) {
    return true;
  }

  bool axes_non_zero = (axes.lx != 0) || (axes.ly != 0) || (axes.rx != 0) || (axes.ry != 0);
  if (!axes_non_zero && (now - g_last_compact_log_ms) >= COMPACT_IDLE_HEARTBEAT_MS) {
    return true;
  }

  return false;
}

static void log_compact_state(uint8_t const* report, uint16_t len, AxisState const& axes) {
  Serial.print(F("STATE #"));
  Serial.print(g_report_count);
  Serial.print(F(" lx="));
  Serial.print(axes.lx);
  Serial.print(F(" ly="));
  Serial.print(axes.ly);
  Serial.print(F(" rx="));
  Serial.print(axes.rx);
  Serial.print(F(" ry="));
  Serial.print(axes.ry);

  if (LOG_BUTTON_BYTES && len > BUTTON_BYTE_FIRST) {
    uint8_t btn_last = BUTTON_BYTE_LAST;
    if (btn_last >= len) {
      btn_last = (uint8_t)(len - 1);
    }
    for (uint8_t b = BUTTON_BYTE_FIRST; b <= btn_last; b++) {
      Serial.print(F(" b"));
      Serial.print(b);
      Serial.print(F("=0x"));
      print_hex_byte(report[b]);
    }
  }
  Serial.println();

  g_last_compact_axes = axes;
  g_have_last_compact_axes = true;
  g_last_compact_log_ms = millis();

  if (LOG_BUTTON_BYTES && len > BUTTON_BYTE_FIRST) {
    uint8_t btn_last = BUTTON_BYTE_LAST;
    if (btn_last >= len) {
      btn_last = (uint8_t)(len - 1);
    }
    uint8_t idx = 0;
    for (uint8_t b = BUTTON_BYTE_FIRST; b <= btn_last; b++, idx++) {
      g_last_compact_buttons[idx] = report[b];
    }
    g_have_last_compact_buttons = true;
  }
}

static void update_stats(uint8_t const* report, uint16_t len) {
  uint16_t n = len > MAX_REPORT_LEN ? MAX_REPORT_LEN : len;
  for (uint16_t i = 0; i < n; i++) {
    uint8_t v = report[i];
    if (v < g_min_byte[i]) {
      g_min_byte[i] = v;
    }
    if (v > g_max_byte[i]) {
      g_max_byte[i] = v;
    }
    g_or_mask[i] |= v;
    g_and_mask[i] &= v;
  }
}

static uint8_t byte_span(uint16_t idx) {
  return (uint8_t)(g_max_byte[idx] - g_min_byte[idx]);
}

static void print_activity_summary(uint16_t len) {
  uint16_t n = len > MAX_REPORT_LEN ? MAX_REPORT_LEN : len;
  uint8_t top_idx[6];
  uint8_t top_span[6];
  uint8_t top_count = 0;
  uint8_t active_count = 0;

  for (uint16_t i = 0; i < n; i++) {
    uint8_t span = byte_span(i);
    if (span == 0) {
      continue;
    }
    active_count++;

    uint8_t pos = top_count;
    if (top_count < 6) {
      top_count++;
    }
    while (pos > 0 && top_span[pos - 1] < span) {
      if (pos < 6) {
        top_span[pos] = top_span[pos - 1];
        top_idx[pos] = top_idx[pos - 1];
      }
      pos--;
    }
    if (pos < 6) {
      top_span[pos] = span;
      top_idx[pos] = (uint8_t)i;
    }
  }

  Serial.print(F("SUMMARY reports="));
  Serial.print(g_report_count);
  Serial.print(F(" changed="));
  Serial.print(g_change_count);
  Serial.print(F(" unchanged="));
  Serial.print(g_unchanged_count);
  Serial.print(F(" filtered="));
  Serial.print(g_filtered_report_count);
  Serial.print(F(" active_bytes="));
  Serial.println(active_count);

  if (active_count == 0) {
    Serial.println(F("SUMMARY no changing bytes yet. Move one stick/button at a time."));
    return;
  }

  Serial.println(F("TOP bytes by span (index min max span and bit-flip mask):"));
  for (uint8_t i = 0; i < top_count; i++) {
    uint8_t idx = top_idx[i];
    Serial.print(F("  ["));
    Serial.print(idx);
    Serial.print(F("] min=0x"));
    print_hex_byte(g_min_byte[idx]);
    Serial.print(F(" max=0x"));
    print_hex_byte(g_max_byte[idx]);
    Serial.print(F(" span="));
    Serial.print(top_span[i]);
    Serial.print(F(" toggles=0x"));
    print_hex_byte((uint8_t)(g_or_mask[idx] ^ g_and_mask[idx]));
    Serial.println();
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  uint32_t start = millis();
  while (!Serial && (millis() - start) < 2500) {
    delay(10);
  }

  Serial.println();
  Serial.println(F("=== USB host gamepad probe ==="));
  Serial.println(F("Move one control at a time for easy mapping."));
  if (LOG_BUTTON_LIST_1HZ) {
    Serial.print(F("Default output is pressed button list at "));
    Serial.print(BUTTON_LIST_LOG_INTERVAL_MS);
    Serial.println(F(" ms intervals."));
  } else if (LOG_COMPACT_STATE) {
    Serial.println(F("Default output is throttled STATE lines (axes + optional button bytes)."));
  } else {
    Serial.println(F("STATE output disabled (button-mapping mode)."));
  }
  Serial.println(F("Set LOG_RAW_REPORTS/LOG_DELTA_LINES=true for full raw debug."));
  if (REPORT_ID_FILTER_ENABLED) {
    Serial.print(F("Report ID filter enabled: rid=0x"));
    print_hex_byte(REPORT_ID_FILTER);
    Serial.println();
  } else {
    Serial.println(F("Report ID filter disabled (all report IDs processed)."));
  }
  if (LOG_BUTTON_BYTES) {
    Serial.println(F("Button candidate bytes are included in STATE (b23..b26)."));
  } else {
    Serial.println(F("Button candidate bytes are disabled (set LOG_BUTTON_BYTES=true to include b23..b26)."));
  }
  if (LOG_BUTTON_EVENTS) {
    Serial.print(F("Button event mapper on bytes ["));
    Serial.print(BUTTON_EVENT_BYTE_FIRST);
    Serial.print(F(".."));
    Serial.print(BUTTON_EVENT_BYTE_LAST);
    Serial.print(F("], stable_reports="));
    Serial.print(BUTTON_EVENT_STABLE_REPORTS);
    Serial.print(F(", min_interval_ms="));
    Serial.print(BUTTON_EVENT_MIN_INTERVAL_MS);
    Serial.print(F(", max_toggled_bits="));
    Serial.println(BUTTON_EVENT_MAX_TOGGLED_BITS);
  } else {
    Serial.println(F("Button event mapper disabled (set LOG_BUTTON_EVENTS=true)."));
  }
  if (robot_config::kUsbHostMapToDrive) {
    Serial.println(F("Drive control enabled: hold RB to drive, press A to stop."));
    Serial.println(F("Y zeros steer encoders and saves offsets to EEPROM. B is the horn."));
  } else {
    Serial.println(F("Drive control disabled by config."));
  }
  robot_config::beginRobotHardware();
  g_robot_hardware_started = true;
  Serial.println(F("Robot hardware initialized."));
  Serial.print(F("Steer offset load status: "));
  Serial.println(robot_config::lastSteerOffsetLoadStatus());
  Serial.println(F("Hold GP21 during boot to run startup autotune."));
  maybe_run_startup_autotune();
  Serial.println(F("Waiting for USB host core startup..."));

  reset_report_tracking();
  g_last_wait_log_ms = millis();
}

void loop() {
  uint32_t now = millis();
  log_steer_encoder_debug(now);
  service_swerve_drive(now);
  service_disconnect_alarm(millis());

  if (!g_host_started) {
    if (now - g_last_wait_log_ms >= 3000) {
      Serial.println(F("USB host is not running."));
      g_last_wait_log_ms = now;
    }
    stop_drive_outputs();
    delay(25);
    return;
  }

  if (!g_device_mounted) {
    if (now - g_last_wait_log_ms >= 3000) {
      Serial.println(F("USB host waiting for a gamepad or dongle..."));
      g_last_wait_log_ms = now;
    }
    stop_drive_outputs();
    delay(25);
    return;
  }

  const uint32_t last_valid_report_ms = g_last_valid_report_ms;
  const uint32_t log_now = millis();
  if ((log_now - last_valid_report_ms) >= HOST_NO_ACTIVE_REPORT_LOG_MS &&
      (log_now - g_last_no_report_log_ms) >= HOST_NO_ACTIVE_REPORT_LOG_MS) {
    Serial.println(F("USB host: controller mounted, but no recent valid gamepad reports."));
    g_last_no_report_log_ms = log_now;
  }

  delay(2);
}

// Core 1 runs the USB host stack.
void setup1() {
  rp2040_configure_pio_usb();
  bool begin_ok = USBHost.begin(HOST_RHPORT);
  g_host_started = begin_ok;
  if (begin_ok) {
    Serial.println(F("USB host init complete"));
  } else {
    Serial.println(F("USB host init failed. Check the USB host configuration and reset the board."));
  }
}

static void handle_hid_instance_unmounted(uint8_t dev_addr, uint8_t instance, bool callback_reported) {
  if (callback_reported) {
    Serial.print(F("UNMOUNT addr="));
  } else {
    Serial.print(F("UNMOUNT missed-callback addr="));
  }
  Serial.print(dev_addr);
  Serial.print(F(" instance="));
  Serial.println(instance);

  if (dev_addr == g_dev_addr) {
    uint8_t instance_bit = hid_instance_mask(instance);
    const bool was_tracked = (g_mounted_instances & instance_bit) != 0;
    g_mounted_instances &= (uint8_t)~instance_bit;

    if (g_have_active_instance && instance == g_active_instance) {
      g_have_active_instance = false;
      g_active_instance = 0;
      reset_report_tracking();
    }

    if (g_mounted_instances == 0 || instance_bit == 0) {
      g_device_mounted = false;
      g_dev_addr = 0;
      g_mounted_instances = 0;
      g_have_active_instance = false;
      g_active_instance = 0;
      if (was_tracked) {
        g_usb_disconnect_pending.store(true, std::memory_order_relaxed);
      }
    }
  }

  g_last_wait_log_ms = millis();
}

static void reconcile_mounted_hid_instances() {
  if (!g_device_mounted || g_dev_addr == 0) {
    return;
  }

  uint8_t dev_addr = g_dev_addr;
  uint8_t instances = g_mounted_instances;

  for (uint8_t instance = 0; instance < MAX_HID_INSTANCES_TRACKED; instance++) {
    uint8_t instance_bit = hid_instance_mask(instance);
    if ((instances & instance_bit) == 0) {
      continue;
    }
    if (!tuh_hid_mounted(dev_addr, instance)) {
      handle_hid_instance_unmounted(dev_addr, instance, false);
    }
  }
}

static void requeue_mounted_hid_reports() {
  if (!g_device_mounted || g_dev_addr == 0) {
    return;
  }

  uint32_t now = millis();
  if ((now - g_last_mount_ms) < HOST_MOUNT_SETTLE_MS) {
    return;
  }

  if ((now - g_last_report_requeue_ms) < HOST_REPORT_REQUEUE_INTERVAL_MS) {
    return;
  }
  g_last_report_requeue_ms = now;

  uint8_t dev_addr = g_dev_addr;
  uint8_t instances = g_mounted_instances;

  for (uint8_t instance = 0; instance < MAX_HID_INSTANCES_TRACKED; instance++) {
    uint8_t instance_bit = hid_instance_mask(instance);
    if ((instances & instance_bit) == 0) {
      continue;
    }
    if (g_have_active_instance && instance != g_active_instance) {
      continue;
    }
    if (tuh_hid_mounted(dev_addr, instance) && tuh_hid_receive_ready(dev_addr, instance)) {
      tuh_hid_receive_report(dev_addr, instance);
    }
  }
}

void loop1() {
  if (g_host_started) {
    USBHost.task(HOST_TASK_TIMEOUT_MS, false);
    reconcile_mounted_hid_instances();
    requeue_mounted_hid_reports();
  } else {
    delay(2);
  }
}

extern "C" {
void tuh_hid_mount_cb(uint8_t dev_addr,
                      uint8_t instance,
                      uint8_t const* desc_report,
                      uint16_t desc_len) {
  (void)desc_report;
  (void)desc_len;

  uint8_t instance_bit = hid_instance_mask(instance);
  bool new_device = !g_device_mounted || dev_addr != g_dev_addr;
  g_last_mount_ms = millis();

  if (new_device) {
    reset_report_tracking();
    g_mounted_instances = 0;
    g_active_instance = 0;
    g_have_active_instance = false;
    g_last_valid_report_ms = g_last_mount_ms;
  }

  g_device_mounted = true;
  g_dev_addr = dev_addr;
  g_mounted_instances |= instance_bit;
  g_last_report_requeue_ms = 0;
  uint16_t vid = 0;
  uint16_t pid = 0;
  tuh_vid_pid_get(dev_addr, &vid, &pid);
  g_vid = vid;
  g_pid = pid;

  Serial.print(F("MOUNT addr="));
  Serial.print(dev_addr);
  Serial.print(F(" instance="));
  Serial.print(instance);
  if (instance_bit == 0) {
    Serial.print(F(" ignored=max-instance-tracking-exceeded"));
  }
  Serial.print(F(" VID=0x"));
  print_hex_byte((uint8_t)(g_vid >> 8));
  print_hex_byte((uint8_t)(g_vid & 0xFF));
  Serial.print(F(" PID=0x"));
  print_hex_byte((uint8_t)(g_pid >> 8));
  print_hex_byte((uint8_t)(g_pid & 0xFF));
  Serial.println();

  // Report requests are queued from loop1 after a short settle delay. Some
  // controllers remount before the interrupt endpoint is ready after hotplug.
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  handle_hid_instance_unmounted(dev_addr, instance, true);
}

void tuh_hid_report_received_cb(uint8_t dev_addr,
                                uint8_t instance,
                                uint8_t const* report,
                                uint16_t len) {
  uint8_t instance_bit = hid_instance_mask(instance);
  if (!g_device_mounted ||
      dev_addr != g_dev_addr ||
      instance_bit == 0 ||
      (g_mounted_instances & instance_bit) == 0) {
    return;
  }

  uint16_t n = len > MAX_REPORT_LEN ? MAX_REPORT_LEN : len;
  uint32_t now = millis();
  bool valid_gamepad_report = is_valid_gamepad_report(report, n);

  if (!g_have_active_instance && valid_gamepad_report) {
    g_active_instance = instance;
    g_have_active_instance = true;
    reset_report_tracking();
    now = millis();
  }

  if (g_have_active_instance && instance != g_active_instance) {
    return;
  }

  if (REPORT_ID_FILTER_ENABLED && n > 0 && report[0] != REPORT_ID_FILTER) {
    g_filtered_report_count++;
    if (!tuh_hid_receive_report(dev_addr, instance)) {
      Serial.println(F("ERROR receive_report requeue failed (filtered report)"));
    }
    return;
  }

  g_report_count++;

  if (SKIP_MIRRORED_RID0 && n > 1 && g_have_last_report && g_last_len == n) {
    bool same_payload = (memcmp(report + 1, g_last_report + 1, n - 1) == 0);
    bool rid_flip = (report[0] != g_last_report[0]);
    bool mirrored_rid0 = (report[0] == 0x00) && same_payload && rid_flip;
    if (mirrored_rid0) {
      g_unchanged_count++;
      if (!tuh_hid_receive_report(dev_addr, instance)) {
        Serial.println(F("ERROR receive_report requeue failed"));
      }
      return;
    }
  }

  if (valid_gamepad_report) {
    g_last_valid_report_ms = now;
    update_drive_command_from_report(report, n, now);
  }

  if (LOG_BUTTON_LIST_1HZ &&
      valid_gamepad_report &&
      (now - g_last_button_list_log_ms) >= BUTTON_LIST_LOG_INTERVAL_MS) {
    log_button_list(report, n);
    g_last_button_list_log_ms = now;
  }

  bool changed = true;
  if (g_have_last_report && n == g_last_len) {
    changed = (memcmp(report, g_last_report, n) != 0);
  }

  if (changed) {
    g_change_count++;
    bool axes_changed = range_changed(report, n, 9, 12);
    bool buttons_changed = LOG_BUTTON_BYTES ? range_changed(report, n, BUTTON_BYTE_FIRST, BUTTON_BYTE_LAST) : false;
    bool interesting_changed = axes_changed || buttons_changed;

    if (LOG_RAW_REPORTS && (!LOG_COMPACT_STATE || interesting_changed)) {
      log_report_line(report, n);
    }
    if (LOG_DELTA_LINES && g_have_last_report && (!LOG_COMPACT_STATE || interesting_changed)) {
      log_deltas(report, n);
    }
  } else {
    g_unchanged_count++;
    if (!LOG_BUTTON_LIST_1HZ && (g_unchanged_count % UNCHANGED_HEARTBEAT_N) == 0) {
      Serial.print(F("HEARTBEAT unchanged reports="));
      Serial.println(g_unchanged_count);
    }
  }

  if (LOG_COMPACT_STATE) {
    AxisState axes;
    if (should_log_compact_state(report, n, now, &axes)) {
      log_compact_state(report, n, axes);
    }
  }

  process_button_events(report, n, now);

  update_stats(report, n);

  memcpy(g_last_report, report, n);
  g_last_len = n;
  g_have_last_report = true;

  if (!LOG_BUTTON_LIST_1HZ && (now - g_last_stats_ms) >= STATS_INTERVAL_MS) {
    print_activity_summary(n);
    g_last_stats_ms = now;
  }

  if (!tuh_hid_receive_report(dev_addr, instance)) {
    Serial.println(F("ERROR receive_report requeue failed"));
  }
}
}  // extern "C"
