// SPDX-FileCopyrightText: 2026 Robot project contributors
//
// SPDX-License-Identifier: MIT

#ifndef ROBOT_ENCODER_H
#define ROBOT_ENCODER_H

#include <Arduino.h>

namespace robot {

class Encoder {
 public:
  virtual ~Encoder() {}

  virtual bool begin() { return true; }
  virtual float getAngleDeg() = 0;
  virtual float getAngularVelocityDegS() = 0;
  virtual long getCounts() { return 0; }
  virtual void setCounts(long value) { (void)value; }
  virtual void zero() { setCounts(0); }
  virtual void setOffsetDeg(float offset_deg) { (void)offset_deg; }
  virtual float offsetDeg() const { return 0.0f; }
};

class NullEncoder : public Encoder {
 public:
  explicit NullEncoder(float offset_deg = 0.0f);

  bool begin() override;
  float getAngleDeg() override;
  float getAngularVelocityDegS() override;
  long getCounts() override;
  void setCounts(long value) override;
  void zero() override;
  void setOffsetDeg(float offset_deg) override;
  float offsetDeg() const override;

 private:
  float offset_deg_;
};

class EncoderBase : public Encoder {
 public:
  EncoderBase(float counts_per_rev,
              float offset_deg = 0.0f,
              bool inverted = false,
              bool wrap_velocity = true,
              float vel_alpha = 0.0f,
              float vel_deadband = 0.0f);

  float getAngleDeg() override;
  float getAngularVelocityDegS() override;
  long getCounts() override;
  void setCounts(long value) override;
  void zero() override;
  void setOffsetDeg(float offset_deg) override;
  float offsetDeg() const override;

 protected:
  virtual float readCountsRaw() = 0;

  float counts_per_rev_;
  bool inverted_;
  float offset_counts_;

 private:
  bool wrap_velocity_;
  float vel_alpha_;
  float vel_deadband_;
  bool have_last_sample_;
  float last_counts_;
  uint32_t last_us_;
  float last_velocity_;
};

class IncrementalEncoder : public EncoderBase {
 public:
  IncrementalEncoder(uint8_t pin_a,
                     uint8_t pin_b,
                     float pulses_per_rev,
                     bool inverted = false,
                     float offset_deg = 0.0f,
                     uint8_t interrupt_slot = 0);

  bool begin() override;
  void setCounts(long value) override;

 protected:
  float readCountsRaw() override;

 private:
  uint8_t pin_a_;
  uint8_t pin_b_;
  uint8_t interrupt_slot_;
  volatile long* counter_;
};

class AbsoluteEncoder : public EncoderBase {
 public:
  AbsoluteEncoder(uint8_t pin,
                  float counts_per_rev = 65536.0f,
                  bool inverted = false,
                  float offset_deg = 0.0f,
                  float raw_alpha = 0.25f,
                  float vel_alpha = 0.25f,
                  float vel_deadband = 0.5f);

  bool begin() override;
  int readRaw();
  int adcMax() const;

 protected:
  float readCountsRaw() override;

 private:
  uint8_t pin_;
  int adc_max_;
  float raw_alpha_;
  bool have_raw_ema_;
  float raw_ema_;
};

}  // namespace robot

#endif  // ROBOT_ENCODER_H
