/*
 * Copyright (C) 2024 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file widget.hpp
 *
 * @brief Header file for the WidgetModule class
 *
 * @authors Your Name
 * Contact: your.email@unmanned.life
 *
 */
#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_WIDGET_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_WIDGET_HPP_

#include <dji_widget.h>
#include <dji_error.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <string>
#include <vector>
#include <shared_mutex>
#include <nlohmann/json.hpp>

#include "psdk_interfaces/msg/widget_press.hpp"


namespace psdk_ros2
{

class WidgetModule : public rclcpp_lifecycle::LifecycleNode
{
 public:
  /**
   * @brief Construct a new WidgetModule object
   * @param node_name Name of the node
   */
  explicit WidgetModule(const std::string& name);

  /**
   * @brief Destroy the WidgetModule object
   */
  ~WidgetModule();

  /**
   * @brief Configures the WidgetModule. Creates the ROS 2 subscribers
   * and services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state);

  /**
   * @brief Activates the WidgetModule.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state);

  /**
   * @brief Cleans the WidgetModule. Resets the ROS 2 subscribers and
   * services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state);

  /**
   * @brief Deactivates the WidgetModule.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state);

  /**
   * @brief Shuts down the WidgetModule.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state);

  /**
   * @brief Initialize the widget module
   * @note This method should be invoked after ROS 2 elements are initialized.
   * @return true/false
   */
  bool init();

  /**
   * @brief Deinitialize the widget module
   * @return true/false
   */
  bool deinit();
  
  std::string widget_config_file_path_;

private:
  // Widget handling
  T_DjiReturnCode SetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value);
  T_DjiReturnCode GetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value);

  // Static wrappers
  static T_DjiReturnCode StaticSetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value, void *userData);
  static T_DjiReturnCode StaticGetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value, void *userData);

  // Members
  std::vector<int32_t> widget_value_list_;
  std::vector<T_DjiWidgetHandlerListItem> widget_handler_list_;
  rclcpp_lifecycle::LifecyclePublisher<psdk_interfaces::msg::WidgetPress>::SharedPtr widget_value_publisher_;
  bool is_module_initialized_{false};
  mutable std::shared_mutex global_ptr_mutex_;

};

extern std::shared_ptr<WidgetModule> global_widget_ptr_;

}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_WIDGET_HPP_
