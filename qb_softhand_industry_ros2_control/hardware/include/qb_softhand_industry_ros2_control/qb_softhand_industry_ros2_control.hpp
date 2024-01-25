// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef QB_SOFTHAND_INDUSTRY_ROS2_CONTROL_HPP_
#define QB_SOFTHAND_INDUSTRY_ROS2_CONTROL_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/state.hpp"
#include "transmission_interface/transmission.hpp"

#include <qb_softhand_industry_msgs/msg/resource_data.hpp>
#include <qb_softhand_industry_msgs/msg/state.hpp>
#include <qb_softhand_industry_msgs/msg/state_stamped.hpp>
#include <qb_softhand_industry_srvs/srv/get_measurements.hpp>
#include <qb_softhand_industry_srvs/srv/set_command.hpp>
#include <qb_softhand_industry_srvs/srv/set_commands.hpp>
#include <qb_softhand_industry_srvs/srv/trigger.hpp>

namespace qb_softhand_industry_ros2_control
{
class QbSofthandIndustryHW : public hardware_interface::SystemInterface
{
public:

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

private:
  std::unique_ptr<rclcpp::Logger> logger_;
  std::unique_ptr<rclcpp::Clock> clock_;

  std::shared_ptr<rclcpp::Node> node_;

  int max_repeats_ = 0;
  std::string device_name_;
  // parameters for the RRBot simulation
  double actuator_slowdown_;

  // transmissions
  std::vector<std::shared_ptr<transmission_interface::Transmission>> transmissions_;

  struct InterfaceData
  {
    explicit InterfaceData(const std::string & name);

    std::string name_;
    double command_;
    double state_;

    // this is the "sink" that will be part of the transmission Joint/Actuator handles
    double transmission_passthrough_;
  };
  std::vector<InterfaceData> joint_interfaces_;
  std::vector<InterfaceData> actuator_interfaces_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;

  rclcpp::Client<qb_softhand_industry_srvs::srv::GetMeasurements>::SharedPtr get_measurements_client_;
  rclcpp::Client<qb_softhand_industry_srvs::srv::SetCommand>::SharedPtr set_command_client_;
  rclcpp::Client<qb_softhand_industry_srvs::srv::SetCommands>::SharedPtr set_commands_client_;
  rclcpp::Client<qb_softhand_industry_srvs::srv::Trigger>::SharedPtr activate_client_;
  rclcpp::Client<qb_softhand_industry_srvs::srv::Trigger>::SharedPtr deactivate_client_;
  
  //Initialize the services needed for getting/setting cmds from/to the device
  void initializeServicesAndWait();

  //Wait untill all the necessary servers exists
  void waitForSrvs();
     
  //Set commands to control qb SoftHand Industry 
  bool setCommands(float &position, float &velocity, float &current);

   
  // Get the Measurements from the device, by advertising the server 
  // return true if the response of the server is true, false otherwise 
  bool getMeasurements(float &position, float &velocity, float &current, rclcpp::Time & stamp);
};

}  // namespace qb_softhand_industry_ros2_control

#endif  // QB_SOFTHAND_INDUSTRY_ROS2_CONTROL_HPP_
