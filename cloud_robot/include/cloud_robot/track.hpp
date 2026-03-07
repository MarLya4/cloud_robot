#pragma once

#include <string>

namespace cloud_robot
{

struct TrackConfig
{
  std::string name;

  int pigpio_handle;

  int slp_gpio;
  int dir_gpio;
  int pwm_gpio;

  double rpm_motor;
  double gear_ratio;
  double max_accel;

  bool invert_direction;
};

class Track
{
public:

  explicit Track(const TrackConfig& config);

  bool enable();
  bool disable();

  void update(double target_rad_s, double dt);

  double get_velocity() const;

  // getters seguros
  double get_max_rad_s() const;
  double get_rpm_motor() const;
  double get_max_accel() const;

private:

  void set_motor_output(double norm);

  // config
  std::string name_;

  int pigpio_handle_;

  int slp_gpio_;
  int dir_gpio_;
  int pwm_gpio_;

  bool invert_direction_;

  double rpm_motor_;
  double gear_ratio_;
  double track_max_rad_s_;
  double max_accel_;

  // state
  double current_rad_s_;
  double prev_target_;
};

} // namespace cloud_robot
