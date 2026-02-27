#include "cloud_hardware/track.hpp"

#include <pigpiod_if2.h>
#include <algorithm>
#include <cmath>

namespace cloud_hardware
{

Track::Track(const TrackConfig& config)
: name_(config.name),
  pigpio_handle_(config.pigpio_handle),
  slp_gpio_(config.slp_gpio),
  dir_gpio_(config.dir_gpio),
  pwm_gpio_(config.pwm_gpio),
  invert_direction_(config.invert_direction),
  rpm_motor_(config.rpm_motor),
  gear_ratio_(config.gear_ratio),
  max_accel_(config.max_accel),
  current_rad_s_(0.0),
  prev_target_(0.0)
{
  const double motor_rad_s = rpm_motor_ * 2.0 * M_PI / 60.0;
  track_max_rad_s_ = motor_rad_s / gear_ratio_;
}

bool Track::enable()
{
  set_mode(pigpio_handle_, slp_gpio_, PI_OUTPUT);
  set_mode(pigpio_handle_, dir_gpio_, PI_OUTPUT);
  set_mode(pigpio_handle_, pwm_gpio_, PI_OUTPUT);

  gpio_write(pigpio_handle_, slp_gpio_, 1);

  set_PWM_range(pigpio_handle_, pwm_gpio_, 255);
  set_PWM_frequency(pigpio_handle_, pwm_gpio_, 20000);

  return true;
}

bool Track::disable()
{
 set_PWM_dutycycle(pigpio_handle_, pwm_gpio_, 0);
 gpio_write(pigpio_handle_, slp_gpio_, 0);
  current_rad_s_ = 0.0;
  prev_target_ = 0.0;

  return true;
}

void Track::update(double target_rad_s, double dt)
{
  target_rad_s = std::clamp(
      target_rad_s,
      -track_max_rad_s_,
       track_max_rad_s_);

  double max_delta = max_accel_ * dt;
  double delta = target_rad_s - current_rad_s_;
  delta = std::clamp(delta, -max_delta, max_delta);

  current_rad_s_ += delta;

  double norm = current_rad_s_ / track_max_rad_s_;
  norm = std::clamp(norm, -1.0, 1.0);

  set_motor_output(norm);
}

void Track::set_motor_output(double norm)
{
  if (invert_direction_)
    norm = -norm;

  bool direction = norm >= 0.0;

  gpio_write(pigpio_handle_, dir_gpio_, direction);

  double duty = std::abs(norm) * 255.0;

  set_PWM_dutycycle(
    pigpio_handle_,
    pwm_gpio_,
    static_cast<int>(duty));
}

double Track::get_velocity() const
{
  return current_rad_s_;
}

double Track::get_max_rad_s() const
{
  return track_max_rad_s_;
}

double Track::get_rpm_motor() const
{
  return rpm_motor_;
}

double Track::get_max_accel() const
{
  return max_accel_;
}

} // namespace cloud_hardware
