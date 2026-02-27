#include "cloud_hardware/servo_hw.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <algorithm>
#include <vector>

namespace cloud_hardware
{

hardware_interface::CallbackReturn ServoHW::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  size_t n = info.joints.size();

  servos_.resize(n);
  current_pos_.assign(n, 0.0);
  goal_pos_.assign(n, 0.0);

  for (size_t i = 0; i < n; i++)
  {
    const auto & joint = info.joints[i];

    servos_[i].joint_name = joint.name;

    servos_[i].channel =
      std::stoi(joint.parameters.at("channel"));

    servos_[i].rad_min =
      std::stod(joint.parameters.at("rad_min"));

    servos_[i].rad_max =
      std::stod(joint.parameters.at("rad_max"));

    servos_[i].us_min =
      std::stoi(joint.parameters.at("us_min"));

    servos_[i].us_max =
      std::stoi(joint.parameters.at("us_max"));

    servos_[i].zero_offset_rad =
      std::stod(joint.parameters.at("zero_offset_rad"));

    servos_[i].invert =
      joint.parameters.at("invert") == "true";

    RCLCPP_INFO(
      rclcpp::get_logger("ServoHW"),
      "Loaded joint %s | channel=%d | invert=%d",
      servos_[i].joint_name.c_str(),
      servos_[i].channel,
      servos_[i].invert);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ServoHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < current_pos_.size(); i++)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      "position",
      &current_pos_[i]);
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
ServoHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < goal_pos_.size(); i++)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      "position",
      &goal_pos_[i]);
  }

  return command_interfaces;
}



hardware_interface::CallbackReturn ServoHW::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!driver_connect())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ServoHW"),
      "Failed to connect servo driver");

    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ServoHW"),
    "Servo driver connected");

  // mover a posición inicial (0 rad conceptual)
  for (size_t i = 0; i < servos_.size(); i++)
  {
    goal_pos_[i] = 0.0;
    servo_direct(i, goal_pos_[i]);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn ServoHW::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ServoHW"),
    "Deactivating servos");

  // volver a posición inicial
  for (size_t i = 0; i < servos_.size(); i++)
  {
    goal_pos_[i] = 0.0;
    servo_direct(i, goal_pos_[i]);
  }

  driver_disconnect();

  return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type ServoHW::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  for (size_t i = 0; i < servos_.size(); i++)
  {
    current_pos_[i] = goal_pos_[i];
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type ServoHW::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  for (size_t i = 0; i < servos_.size(); i++)
  {
    servo_direct(i, goal_pos_[i]);
  }

  return hardware_interface::return_type::OK;
}


bool ServoHW::driver_connect()
{
  port_ = "/dev/ttyACM0";

  serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);

  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ServoHW"),
      "Cannot open port %s",
      port_.c_str());

    return false;
  }

  return true;
}


bool ServoHW::driver_disconnect()
{
  if (serial_fd_ >= 0)
  {
    close(serial_fd_);
    serial_fd_ = -1;
  }

  return true;
}


bool ServoHW::driver_send(const std::vector<uint8_t>& data)
{
  if (serial_fd_ < 0)
    return false;

  ssize_t written = write(serial_fd_, data.data(), data.size());

  return written == static_cast<ssize_t>(data.size());
}


bool ServoHW::set_servo_us(uint8_t channel, uint16_t us)
{
  uint16_t target = us * 4;

  std::vector<uint8_t> cmd =
  {
    0x84,
    channel,
    static_cast<uint8_t>(target & 0x7F),
    static_cast<uint8_t>((target >> 7) & 0x7F)
  };

  return driver_send(cmd);
}


uint16_t ServoHW::cmd_rad_to_us(
  const Servo & s,
  double rad_cmd)
{
  // aplicar invert conceptual
  if (s.invert)
    rad_cmd = -rad_cmd;

  // limitar rango conceptual
  rad_cmd = std::clamp(rad_cmd, s.rad_min, s.rad_max);

  // convertir a rango físico real
  double rad_real = rad_cmd + s.zero_offset_rad;

  // normalizar
  double norm =
    rad_real /
    (s.rad_max - s.rad_min);

  uint16_t us =
    s.us_min +
    norm * (s.us_max - s.us_min);

  RCLCPP_DEBUG(
    rclcpp::get_logger("ServoHW"),
    "%s | rad=%.2f | us=%d",
    s.joint_name.c_str(),
    rad_cmd,
    us);

  return us;
}

void ServoHW::servo_direct(
  size_t i,
  double target_rad)
{
  uint16_t us =
    cmd_rad_to_us(servos_[i], target_rad);

  set_servo_us(servos_[i].channel, us);

  current_pos_[i] = target_rad;
}


} // namespace cloud_hardware


PLUGINLIB_EXPORT_CLASS(
  cloud_hardware::ServoHW,
  hardware_interface::SystemInterface)
