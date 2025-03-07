
#ifndef ADOG_LEG_HARDWARE_HPP_
#define ADOG_LEG_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace adog_leg_hardware
{
class LegSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LegSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:

  // Store the command for the simulated robot
    std::vector<double> joint_torque_command_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocities_command_;
    std::vector<double> joint_kp_command_;
    std::vector<double> joint_kd_command_;

    std::vector<double> joint_position_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_effort_;

    std::string imu_name_;
    std::vector<std::string> imu_state_names_;
    std::vector<double> imu_states_;

    std::string foot_force_name_;
    std::vector<std::string> foot_force_interfaces_;
    std::vector<double> foot_force_values_;

    std::unordered_map<std::string, std::vector<std::string> > joint_command_interfaces = {
        {"position", {}},
        {"velocity", {}},
        {"effort", {}},
        {"kp", {}},
        {"kd", {}}
    };
    std::unordered_map<std::string, std::vector<std::string> > joint_state_interfaces = {
        {"position", {}},
        {"velocity", {}},
        {"effort", {}},

    };
};

}  // namespace ros2_control_demo_example_12

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_12__RRBOT_HPP_