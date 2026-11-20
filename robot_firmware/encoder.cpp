// SPDX-FileCopyrightText: 2026 Robot project contributors
//
// SPDX-License-Identifier: MIT

#include "encoder.h"

#include <math.h>

namespace robot {
namespace {

constexpr uint8_t kMaxInterruptEncoders = 8;
volatile long g_quad_counts[kMaxInterruptEncoders];
uint8_t g_quad_pin_a[kMaxInterruptEncoders];
uint8_t g_quad_pin_b[kMaxInterruptEncoders];

float wrap360(float value) {
  float wrapped = fmodf(value, 360.0f);
  if (wrapped < 0.0f) {
    wrapped += 360.0f;
  }
  return wrapped;
}

void handleQuadrature(uint8_t slot) {
  bool a = digitalRead(g_quad_pin_a[slot]);
  bool b = digitalRead(g_quad_pin_b[slot]);
  g_quad_counts[slot] += (a == b) ? 1 : -1;
}

void isr0() { handleQuadrature(0); }
void isr1() { handleQuadrature(1); }
void isr2() { handleQuadrature(2); }
void isr3() { handleQuadrature(3); }
void isr4() { handleQuadrature(4); }
void isr5() { handleQuadrature(5); }
void isr6() { handleQuadrature(6); }
void isr7() { handleQuadrature(7); }

void (*isrForSlot(uint8_t slot))() {
  switch (slot) {
    case 0: return isr0;
    case 1: return isr1;
    case 2: return isr2;
    case 3: return isr3;
    case 4: return isr4;
    case 5: return isr5;
    case 6: return isr6;
    case 7: return isr7;
    default: return nullptr;
  }
}

}  // namespace

NullEncoder::NullEncoder(float offset_deg) : offset_deg_(offset_deg) {}

bool NullEncoder::begin() {
  return true;
}

float NullEncoder::getAngleDeg() {
  return wrap360(offset_deg_);
}

float NullEncoder::getAngularVelocityDegS() {
  return 0.0f;
}

long NullEncoder::getCounts() {
  return 0;
}

void NullEncoder::setCounts(long value) {
  offset_deg_ = static_cast<float>(value);
}

void NullEncoder::zero() {
  offset_deg_ = 0.0f;
}

void NullEncoder::setOffsetDeg(float offset_deg) {
  offset_deg_ = offset_deg;
}

float NullEncoder::offsetDeg() const {
  return offset_deg_;
}

EncoderBase::EncoderBase(float counts_per_rev,
                         float offset_deg,
                         bool inverted,
                         bool wrap_velocity,
                         float vel_alpha,
                         float vel_deadband)
    : counts_per_rev_(counts_per_rev > 0.0f ? counts_per_rev : 1.0f),
      inverted_(inverted),
      offset_counts_((offset_deg / 360.0f) * counts_per_rev_),
      wrap_velocity_(wrap_velocity),
      vel_alpha_(vel_alpha),
      vel_deadband_(vel_deadband),
      have_last_sample_(false),
      last_counts_(0.0f),
      last_us_(0),
      last_velocity_(0.0f) {}

float EncoderBase::getAngleDeg() {
  float counts = readCountsRaw();
  return wrap360(((counts + offset_counts_) / counts_per_rev_) * 360.0f);
}

float EncoderBase::getAngularVelocityDegS() {
  uint32_t now_us = micros();
  float counts = readCountsRaw();
  if (!have_last_sample_) {
    have_last_sample_ = true;
    last_counts_ = counts;
    last_us_ = now_us;
    last_velocity_ = 0.0f;
    return 0.0f;
  }

  uint32_t dt_us = now_us - last_us_;
  if (dt_us == 0) {
    return last_velocity_;
  }

  float delta = counts - last_counts_;
  if (wrap_velocity_) {
    float half = counts_per_rev_ * 0.5f;
    delta = fmodf(delta + half, counts_per_rev_);
    if (delta < 0.0f) {
      delta += counts_per_rev_;
    }
    delta -= half;
  }

  float dt_s = static_cast<float>(dt_us) / 1000000.0f;
  float vel = (delta / counts_per_rev_) * 360.0f / dt_s;
  if (vel_alpha_ > 0.0f) {
    vel = last_velocity_ + vel_alpha_ * (vel - last_velocity_);
  }
  if (fabsf(vel) < vel_deadband_) {
    vel = 0.0f;
  }

  last_counts_ = counts;
  last_us_ = now_us;
  last_velocity_ = vel;
  return vel;
}

long EncoderBase::getCounts() {
  return lroundf(readCountsRaw() + offset_counts_);
}

void EncoderBase::setCounts(long value) {
  offset_counts_ = static_cast<float>(value) - readCountsRaw();
}

void EncoderBase::zero() {
  offset_counts_ = -readCountsRaw();
}

void EncoderBase::setOffsetDeg(float offset_deg) {
  offset_counts_ = (offset_deg / 360.0f) * counts_per_rev_;
}

float EncoderBase::offsetDeg() const {
  float offset = fmodf((offset_counts_ / counts_per_rev_) * 360.0f, 360.0f);
  if (offset < 0.0f) {
    offset += 360.0f;
  }
  return offset;
}

IncrementalEncoder::IncrementalEncoder(uint8_t pin_a,
                                       uint8_t pin_b,
                                       float pulses_per_rev,
                                       bool inverted,
                                       float offset_deg,
                                       uint8_t interrupt_slot)
    : EncoderBase(pulses_per_rev, offset_deg, inverted, false, 0.0f, 0.0f),
      pin_a_(pin_a),
      pin_b_(pin_b),
      interrupt_slot_(interrupt_slot),
      counter_(nullptr) {}

bool IncrementalEncoder::begin() {
  if (interrupt_slot_ >= kMaxInterruptEncoders) {
    return false;
  }
  void (*isr)() = isrForSlot(interrupt_slot_);
  if (isr == nullptr) {
    return false;
  }

  pinMode(pin_a_, INPUT_PULLUP);
  pinMode(pin_b_, INPUT_PULLUP);
  g_quad_pin_a[interrupt_slot_] = pin_a_;
  g_quad_pin_b[interrupt_slot_] = pin_b_;
  g_quad_counts[interrupt_slot_] = 0;
  counter_ = &g_quad_counts[interrupt_slot_];
  attachInterrupt(digitalPinToInterrupt(pin_a_), isr, CHANGE);
  return true;
}

void IncrementalEncoder::setCounts(long value) {
  if (counter_ == nullptr) {
    EncoderBase::setCounts(value);
    return;
  }
  noInterrupts();
  *counter_ = inverted_ ? -value : value;
  interrupts();
  offset_counts_ = 0.0f;
}

float IncrementalEncoder::readCountsRaw() {
  if (counter_ == nullptr) {
    return 0.0f;
  }
  noInterrupts();
  long value = *counter_;
  interrupts();
  return inverted_ ? -static_cast<float>(value) : static_cast<float>(value);
}

AbsoluteEncoder::AbsoluteEncoder(uint8_t pin,
                                 float counts_per_rev,
                                 bool inverted,
                                 float offset_deg,
                                 float raw_alpha,
                                 float vel_alpha,
                                 float vel_deadband)
    : EncoderBase(counts_per_rev, offset_deg, inverted, true, vel_alpha, vel_deadband),
      pin_(pin),
      adc_max_(1023),
      raw_alpha_(raw_alpha),
      have_raw_ema_(false),
      raw_ema_(0.0f) {}

bool AbsoluteEncoder::begin() {
#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
  analogReadResolution(12);
  adc_max_ = 4095;
#else
  adc_max_ = 1023;
#endif
  pinMode(pin_, INPUT);
  return true;
}

int AbsoluteEncoder::readRaw() {
  return analogRead(pin_);
}

int AbsoluteEncoder::adcMax() const {
  return adc_max_;
}

float AbsoluteEncoder::readCountsRaw() {
  float raw = static_cast<float>(readRaw());
  float adc_max = static_cast<float>(adc_max_ > 0 ? adc_max_ : 1023);

  if (!have_raw_ema_) {
    have_raw_ema_ = true;
    raw_ema_ = raw;
  }

  float half_adc = (adc_max + 1.0f) * 0.5f;
  float span = adc_max + 1.0f;
  float diff = raw - raw_ema_;
  if (diff > half_adc) {
    diff -= span;
  } else if (diff < -half_adc) {
    diff += span;
  }
  raw_ema_ += raw_alpha_ * diff;
  while (raw_ema_ < 0.0f) {
    raw_ema_ += span;
  }
  while (raw_ema_ >= span) {
    raw_ema_ -= span;
  }

  float counts = (raw_ema_ / adc_max) * counts_per_rev_;
  if (inverted_) {
    counts = counts_per_rev_ - counts;
  }
  counts = fmodf(counts, counts_per_rev_);
  if (counts < 0.0f) {
    counts += counts_per_rev_;
  }
  return counts;
}

}  // namespace robot
