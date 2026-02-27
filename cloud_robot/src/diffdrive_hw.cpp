#include "cloud_hardware/diff_drive_hw.hpp"

#include <pigpiod_if2.h>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

#define LOG_IMPORTANTE(logger, fmt, ...) \
  RCLCPP_INFO(logger, "\033[1;35m" fmt "\033[0m", ##__VA_ARGS__)

namespace cloud_hardware
{

/* ============================== */
/* INIT                          */
/* ============================== */

hardware_interface::CallbackReturn DiffDriveHW::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  size_t num_joints = info_.joints.size();

  positions_.resize(num_joints, 0.0);
  velocities_.resize(num_joints, 0.0);
  commands_.resize(num_joints, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}


/* ============================== */
/* CONFIGURE (CREAR TRACKS)      */
/* ============================== */

hardware_interface::CallbackReturn DiffDriveHW::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = rclcpp::get_logger("DiffDriveHW");

  // conectar pigpio
  pi_ = pigpio_start(NULL, NULL);

  if (pi_ < 0)
  {
    RCLCPP_ERROR(logger, "Failed to connect to pigpiod");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger, "Connected to pigpio");

  /* ---------- LEFT TRACK ---------- */

 TrackConfig left_cfg;

left_cfg.name = "left_track";
left_cfg.pigpio_handle = pi_;

left_cfg.slp_gpio =
  std::stoi(info_.hardware_parameters.at("left_slp_gpio"));

left_cfg.dir_gpio =
  std::stoi(info_.hardware_parameters.at("left_dir_gpio"));

left_cfg.pwm_gpio =
  std::stoi(info_.hardware_parameters.at("left_pwm_gpio"));

left_cfg.rpm_motor =
  std::stod(info_.hardware_parameters.at("rpm_motor"));

left_cfg.gear_ratio =
  std::stod(info_.hardware_parameters.at("gear_ratio"));

left_cfg.max_accel =
  std::stod(info_.hardware_parameters.at("max_accel"));

left_cfg.invert_direction =
  info_.hardware_parameters.at("left_invert_direction") == "true";


  left_track_ = std::make_unique<Track>(left_cfg);


  /* ---------- RIGHT TRACK ---------- */

TrackConfig right_cfg;

right_cfg.name = "right_track";
right_cfg.pigpio_handle = pi_;

right_cfg.slp_gpio =
  std::stoi(info_.hardware_parameters.at("right_slp_gpio"));

right_cfg.dir_gpio =
  std::stoi(info_.hardware_parameters.at("right_dir_gpio"));

right_cfg.pwm_gpio =
  std::stoi(info_.hardware_parameters.at("right_pwm_gpio"));

right_cfg.rpm_motor =
  std::stod(info_.hardware_parameters.at("rpm_motor"));

right_cfg.gear_ratio =
  std::stod(info_.hardware_parameters.at("gear_ratio"));

right_cfg.max_accel =
  std::stod(info_.hardware_parameters.at("max_accel"));

right_cfg.invert_direction =
  info_.hardware_parameters.at("right_invert_direction") == "true";

  right_track_ = std::make_unique<Track>(right_cfg);


  /* ---------- LOG ---------- */

  LOG_IMPORTANTE(logger,
    "LEFT TRACK | RPM: %.1f | MAX RAD/S: %.2f | MAX ACCEL: %.2f",
    left_track_->get_rpm_motor(),
    left_track_->get_max_rad_s(),
    left_track_->get_max_accel());

  LOG_IMPORTANTE(logger,
    "RIGHT TRACK | RPM: %.1f | MAX RAD/S: %.2f | MAX ACCEL: %.2f",
    right_track_->get_rpm_motor(),
    right_track_->get_max_rad_s(),
    right_track_->get_max_accel());

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveHW::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto logger = rclcpp::get_logger("DiffDriveHW");

  left_track_->enable();
  right_track_->enable();

  RCLCPP_INFO(logger, "Tracks enabled");

  return hardware_interface::CallbackReturn::SUCCESS;
}




hardware_interface::CallbackReturn DiffDriveHW::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  auto logger = rclcpp::get_logger("DiffDriveHW");

  left_track_->disable();
  right_track_->disable();

  pigpio_stop(pi_);

  RCLCPP_INFO(logger, "Tracks disabled");

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface>
DiffDriveHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
        info_.joints[i].name, "position", &positions_[i]);

    state_interfaces.emplace_back(
        info_.joints[i].name, "velocity", &velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DiffDriveHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
        info_.joints[i].name, "velocity", &commands_[i]);
  }

  return command_interfaces;
}



hardware_interface::return_type DiffDriveHW::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  velocities_[0] = left_track_->get_velocity();
  velocities_[1] = right_track_->get_velocity();

  return hardware_interface::return_type::OK;
}



hardware_interface::return_type DiffDriveHW::write(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  double dt = period.seconds();

  left_track_->update(commands_[0], dt);
  right_track_->update(commands_[1], dt);

  return hardware_interface::return_type::OK;
}


} // namespace cloud_hardware


PLUGINLIB_EXPORT_CLASS(
  cloud_hardware::DiffDriveHW,
  hardware_interface::SystemInterface)
