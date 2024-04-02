// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
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

#include "dynamixel_hardware/dynamixel_hardware.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hardware
{
constexpr const char * kDynamixelHardware = "DynamixelHardware";
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kGoalCurrentIndex = 2;
constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
constexpr const char * kGoalCurrentItem = "Goal_Current";
constexpr const char * kMovingSpeedItem = "Moving_Speed";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentSpeedItem = "Present_Speed";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";
constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity",
  "Profile_Acceleration",
  "Position_P_Gain",
  "Position_I_Gain",
  "Position_D_Gain",
  "Velocity_P_Gain",
  "Velocity_I_Gain",
};

CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "configure");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "joint_id %d: %d", i, joint_ids_[i]);

    if(info_.joints[i].parameters.at("protocol") == "TTL"){
      joint_ids_ttl_.push_back(joint_ids_[i]);
    }else if(info_.joints[i].parameters.at("protocol") == "RS"){
      joint_ids_rs_.push_back(joint_ids_[i]);
    }
  }

  if (
    info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
    (info_.hardware_parameters.at("use_dummy") == "true" || info_.hardware_parameters.at("use_dummy") == "True")) {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "dummy mode");
    return CallbackReturn::SUCCESS;
  }

  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const char * log = nullptr;

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "baud_rate: %d", baud_rate);

  if (!dynamixel_workbench_.init(usb_port.c_str(), baud_rate, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  for (uint i = 0; i < info_.joints.size(); ++i) {
    uint16_t model_number = 0;
    if (!dynamixel_workbench_.ping(joint_ids_[i], &model_number, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return CallbackReturn::ERROR;
    }
  }

  enable_torque(false);
  set_control_mode(control_mode_);
  set_joint_params();
  enable_torque(true);

  const ControlItem * goal_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalPositionItem);
  if (goal_position == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * goal_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalVelocityItem);
  if (goal_velocity == nullptr) {
    goal_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kMovingSpeedItem);
  }
  if (goal_velocity == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * goal_current =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalCurrentItem);
  if (goal_current == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
  if (present_position == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentVelocityItem);
  if (present_velocity == nullptr) {
    present_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentSpeedItem);
  }
  if (present_velocity == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_current =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentCurrentItem);
  if (present_current == nullptr) {
    present_current = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentLoadItem);
  }
  if (present_current == nullptr) {
    return CallbackReturn::ERROR;
  }

  control_items_[kGoalPositionItem] = goal_position;
  control_items_[kGoalVelocityItem] = goal_velocity;
  control_items_[kGoalCurrentItem] = goal_current;
  control_items_[kPresentPositionItem] = present_position;
  control_items_[kPresentVelocityItem] = present_velocity;
  control_items_[kPresentCurrentItem] = present_current;

  if (!dynamixel_workbench_.addSyncWriteHandler(
        control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length,
        &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  if (!dynamixel_workbench_.addSyncWriteHandler(
        control_items_[kGoalVelocityItem]->address, control_items_[kGoalVelocityItem]->data_length,
        &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  if (!dynamixel_workbench_.addSyncWriteHandler(
        control_items_[kGoalCurrentItem]->address, control_items_[kGoalCurrentItem]->data_length,
        &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  uint16_t start_address = std::min(
    control_items_[kPresentPositionItem]->address, control_items_[kPresentCurrentItem]->address);
  uint16_t read_length = control_items_[kPresentPositionItem]->data_length +
                         control_items_[kPresentVelocityItem]->data_length +
                         control_items_[kPresentCurrentItem]->data_length + 2;
  if (!dynamixel_workbench_.addSyncReadHandler(start_address, read_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    for(auto joint_interface:info_.joints[i].state_interfaces){
      if(joint_interface.name == hardware_interface::HW_IF_POSITION){
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
      }
      if(joint_interface.name == hardware_interface::HW_IF_VELOCITY){
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
      }
      if(joint_interface.name == hardware_interface::HW_IF_EFFORT){
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    for(auto joint_interface:info_.joints[i].command_interfaces){
      if(joint_interface.name == hardware_interface::HW_IF_POSITION){
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
      }
      if(joint_interface.name == hardware_interface::HW_IF_VELOCITY){
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
      }
      if(joint_interface.name == hardware_interface::HW_IF_EFFORT){
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].command.effort));
      }
    }
  }

  return command_interfaces;
}


return_type DynamixelHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  std::vector<ControlMode> new_modes;
  if(control_mode_ == ControlMode::CurrentBasedPosition){
    new_modes.push_back(ControlMode::Position);
    new_modes.push_back(ControlMode::Current);
  } else if(control_mode_ != ControlMode::NoControl){
    new_modes.push_back(control_mode_);
  } else { //cotrol_mode_ == ControlMode::NoControl
  }

  std::vector<std::vector<ControlMode>> new_joint_modes(info_.joints.size());
  for (std::size_t i = 0; i < info_.joints.size(); i++){
    for (std::string key : start_interfaces){
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION){
        new_joint_modes[i].push_back(ControlMode::Position);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
        new_joint_modes[i].push_back(ControlMode::Velocity);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT){
        new_joint_modes[i].push_back(ControlMode::Current);
      }
    }
  }
  if (std::all_of(
      new_joint_modes.begin() + 1, new_joint_modes.end(),
      [&](const std::vector<ControlMode>& mode) { return mode == new_joint_modes[0]; })){
    for(auto mode:new_joint_modes[0]){
      new_modes.push_back(mode);
    }
  } else {
    return return_type::ERROR;
  }

  std::vector<std::vector<ControlMode>> stop_joint_modes(info_.joints.size());
  for (std::string key : stop_interfaces){
    for (std::size_t i = 0; i < info_.joints.size(); i++){
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION){
        stop_joint_modes[i].push_back(ControlMode::Position);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
        stop_joint_modes[i].push_back(ControlMode::Velocity);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT){
        stop_joint_modes[i].push_back(ControlMode::Current);
      }
    }
  }
  if (std::all_of(
      stop_joint_modes.begin() + 1, stop_joint_modes.end(),
      [&](const std::vector<ControlMode>& mode) { return mode == stop_joint_modes[0];})){
    for(auto mode:stop_joint_modes[0]){
      auto new_end = std::remove(new_modes.begin(), new_modes.end(), mode);
      new_modes.erase(new_end, new_modes.end());
    }
  } else {
    return return_type::ERROR;
  }

  if(new_modes.size() == 1){
    control_mode_ = new_modes[0];
  }else if(
      std::find(new_modes.begin(), new_modes.end(), ControlMode::Position) != new_modes.end() &&
      std::find(new_modes.begin(), new_modes.end(), ControlMode::Current) != new_modes.end()){
    control_mode_ = ControlMode::CurrentBasedPosition;
  }else if(new_modes.size() == 0){
    RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware), "No Cotrol Interface, Torque disabled");
    control_mode_ = ControlMode::NoControl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Not implemented hardware interfaces");
    return return_type::ERROR;
  }

  return return_type::OK;
}

CallbackReturn DynamixelHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "start");
  for (uint i = 0; i < joints_.size(); i++) {
    if (use_dummy_ && std::isnan(joints_[i].state.position)) {
      joints_[i].state.position = 0.0;
      joints_[i].state.velocity = 0.0;
      joints_[i].state.effort = 0.0;
    }
  }
  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  reset_command();
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "stop");
  return CallbackReturn::SUCCESS;
}

return_type DynamixelHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    return return_type::OK;
  }
  
  if(!joint_ids_ttl_.empty() && !joint_ids_rs_.empty()){
    std::vector<uint8_t>* ids_each[] = {&joint_ids_ttl_, &joint_ids_rs_};
    for(auto& idt: ids_each){
      std::vector<uint8_t> ids(idt->size(), 0);
      std::vector<int32_t> positions(idt->size(), 0);
      std::vector<int32_t> velocities(idt->size(), 0);
      std::vector<int32_t> currents(idt->size(), 0);
      std::copy(idt->begin(), idt->end(), ids.begin());
      const char * log = nullptr;

      if (!dynamixel_workbench_.syncRead(
            kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(), &log)) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      }

      if (!dynamixel_workbench_.getSyncReadData(
            kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
            control_items_[kPresentCurrentItem]->address,
            control_items_[kPresentCurrentItem]->data_length, currents.data(), &log)) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      }

      if (!dynamixel_workbench_.getSyncReadData(
            kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
            control_items_[kPresentVelocityItem]->address,
            control_items_[kPresentVelocityItem]->data_length, velocities.data(), &log)) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      }

      if (!dynamixel_workbench_.getSyncReadData(
            kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
            control_items_[kPresentPositionItem]->address,
            control_items_[kPresentPositionItem]->data_length, positions.data(), &log)) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      }

      for(uint i = 0; i < ids.size(); i++){
        auto it = std::find(joint_ids_.begin(), joint_ids_.end(), ids[i]);
        if(it != joint_ids_.end()){
          int index = std::distance(joint_ids_.begin(), it);
          joints_[index].state.position = dynamixel_workbench_.convertValue2Radian(ids[i], positions[i]);
          joints_[index].state.velocity = dynamixel_workbench_.convertValue2Velocity(ids[i], velocities[i]);
          joints_[index].state.effort = dynamixel_workbench_.convertValue2Current(currents[i]);
        }
      }
    }
  }else{//When TTL and RS485 Protocols are not used at the same time
    std::vector<uint8_t> ids(info_.joints.size(), 0);
    std::vector<int32_t> positions(info_.joints.size(), 0);
    std::vector<int32_t> velocities(info_.joints.size(), 0);
    std::vector<int32_t> currents(info_.joints.size(), 0);

    std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
    const char * log = nullptr;

    if (!dynamixel_workbench_.syncRead(
          kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(), &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }

    if (!dynamixel_workbench_.getSyncReadData(
          kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
          control_items_[kPresentCurrentItem]->address,
          control_items_[kPresentCurrentItem]->data_length, currents.data(), &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }

    if (!dynamixel_workbench_.getSyncReadData(
          kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
          control_items_[kPresentVelocityItem]->address,
          control_items_[kPresentVelocityItem]->data_length, velocities.data(), &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }

    if (!dynamixel_workbench_.getSyncReadData(
          kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
          control_items_[kPresentPositionItem]->address,
          control_items_[kPresentPositionItem]->data_length, positions.data(), &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }

    for (uint i = 0; i < ids.size(); i++) {
      joints_[i].state.position = dynamixel_workbench_.convertValue2Radian(ids[i], positions[i]);
      joints_[i].state.velocity = dynamixel_workbench_.convertValue2Velocity(ids[i], velocities[i]);
      joints_[i].state.effort = dynamixel_workbench_.convertValue2Current(currents[i]);
    }
  }


  return return_type::OK;
}

return_type DynamixelHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    for (auto & joint : joints_) {
      joint.state.position = joint.command.position;
    }
    return return_type::OK;
  }
  
  if(control_mode_ != prev_control_mode_){
    set_control_mode(control_mode_);
    set_joint_params();
    prev_control_mode_ = control_mode_;
  }

  switch (control_mode_) {
    case ControlMode::NoControl:
      return return_type::OK;
      break;
    case ControlMode::Velocity:
      set_joint_velocities();
      return return_type::OK;
      break;
    case ControlMode::Position:
      set_joint_positions();
      return return_type::OK;
      break;
    case ControlMode::Current:
      set_joint_currents();
      return return_type::OK;
      break;
    case ControlMode::CurrentBasedPosition:
      set_joint_currents();
      set_joint_positions();
      return return_type::OK;
      break;
    default: // torque, etc
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Control mode not implemented");
      return return_type::ERROR;
      break;
  }

}

return_type DynamixelHardware::enable_torque(const bool enabled)
{
  const char * log = nullptr;

  if (enabled && !torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOn(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    reset_command();
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque disabled");
  }

  torque_enabled_ = enabled;
  return return_type::OK;
}

return_type DynamixelHardware::set_control_mode(const ControlMode & mode)
{
  const char * log = nullptr;

  if (mode == ControlMode::NoControl) {
    if (torque_enabled_) {
      enable_torque(false);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Position control,but no torque mode");

    return return_type::OK;
  }

  if (mode == ControlMode::Velocity) {
    if (torque_enabled_) {
      enable_torque(false);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setVelocityControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Velocity control");

    enable_torque(true);
    return return_type::OK;
  }

  if (mode == ControlMode::Position) {
    if (torque_enabled_) {
      enable_torque(false);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Position control");

    enable_torque(true);
    return return_type::OK;
  }

  if (mode == ControlMode::Current) {
    if (torque_enabled_) {
      enable_torque(false);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setTorqueControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Current control");

    enable_torque(true);
    return return_type::OK;
  }

  if (mode == ControlMode::CurrentBasedPosition) {
    if (torque_enabled_) {
      enable_torque(false);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setCurrentBasedPositionControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "CurrentBasedPosition control");

    enable_torque(true);
    return return_type::OK;
  }

  if (mode != ControlMode::Velocity && mode != ControlMode::Position && mode != ControlMode::Current && mode != ControlMode::CurrentBasedPosition) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kDynamixelHardware), "Only position/velocity/current/current based position control are implemented");
    return return_type::ERROR;
  }

  return return_type::OK;
}

return_type DynamixelHardware::reset_command()
{
  for (uint i = 0; i < joints_.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.effort = 0.0;
  }

  return return_type::OK;
}

CallbackReturn DynamixelHardware::set_joint_positions()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    commands[i] = dynamixel_workbench_.convertRadian2Value(
      ids[i], static_cast<float>(joints_[i].command.position));
  }
  if (!dynamixel_workbench_.syncWrite(
        kGoalPositionIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_velocities()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    commands[i] = dynamixel_workbench_.convertVelocity2Value(
      ids[i], static_cast<float>(joints_[i].command.velocity));
  }
  if (!dynamixel_workbench_.syncWrite(
        kGoalVelocityIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_currents()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    commands[i] = dynamixel_workbench_.convertCurrent2Value(
      ids[i], static_cast<float>(joints_[i].command.effort));
  }
  if (!dynamixel_workbench_.syncWrite(
        kGoalCurrentIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_params()
{
  const char * log = nullptr;
  for (uint i = 0; i < info_.joints.size(); ++i) {
    for (auto paramName : kExtraJointParameters) {
      if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
        auto value = std::stoi(info_.joints[i].parameters.at(paramName));
        if (!dynamixel_workbench_.itemWrite(joint_ids_[i], paramName, value, &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "%s set to %d for joint %d", paramName, value, i);
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)
