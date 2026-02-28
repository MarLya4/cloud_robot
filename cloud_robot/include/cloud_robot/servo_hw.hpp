#ifndef CLOUD_HARDWARE__SERVO_HW_HPP_
#define CLOUD_HARDWARE__SERVO_HW_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <vector>
#include <string>
#include <cstdint>

namespace cloud_robot
{

class ServoHW : public hardware_interface::SystemInterface
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(ServoHW)


  struct Servo
  {
    std::string joint_name;

    uint8_t channel;

    double rad_min;
    double rad_max;

    uint16_t us_min;
    uint16_t us_max;

    double zero_offset_rad;

    bool invert;
  };



  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;



  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;




  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;


private:



  std::vector<Servo> servos_;

  std::vector<double> current_pos_;
  std::vector<double> goal_pos_;

  std::string port_;
  int serial_fd_ = -1;




  bool driver_connect();

  bool driver_disconnect();

  bool driver_send(const std::vector<uint8_t>& data);

  bool set_servo_us(uint8_t channel, uint16_t us);




  uint16_t cmd_rad_to_us(
    const Servo & s,
    double rad_cmd);

  void servo_direct(
    size_t index,
    double target_rad);

};

} // namespace cloud_hardware

#endif

