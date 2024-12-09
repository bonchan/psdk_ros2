#include "psdk_wrapper/modules/widget.hpp"
namespace psdk_ros2
{

WidgetModule::WidgetModule(const std::string& name)
    : rclcpp_lifecycle::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().arguments(
              {"--ros-args", "-r",
               name + ":" + std::string("__node:=") + name}))
{
  RCLCPP_INFO(get_logger(), "Creating WidgetModule");
}

WidgetModule::~WidgetModule()
{
  RCLCPP_INFO(get_logger(), "Destroying WidgetModule");
  deinit();
}
WidgetModule::CallbackReturn
WidgetModule::on_configure(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Configuring WidgetModule");
  // Add ROS 2 subscriptions, services, or other initialization logic here

  widget_value_publisher_ = this->create_publisher<psdk_interfaces::msg::WidgetPress>(
        "psdk_ros2/widget_press", rclcpp::QoS(10).reliable());

  return CallbackReturn::SUCCESS;
}
WidgetModule::CallbackReturn
WidgetModule::on_activate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Activating WidgetModule");
  // Activate publishers, timers, etc.
  if (widget_value_publisher_) {
        widget_value_publisher_->on_activate();
        RCLCPP_INFO(get_logger(), "Widget value publisher activated.");
    }
  return CallbackReturn::SUCCESS;
}

WidgetModule::CallbackReturn
WidgetModule::on_cleanup(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up WidgetModule");
  // Clean up resources and reset state
  widget_value_publisher_.reset();
  widget_value_list_.clear();
    RCLCPP_INFO(get_logger(), "Internal widget state cleared.");

  return CallbackReturn::SUCCESS;
}

WidgetModule::CallbackReturn
WidgetModule::on_deactivate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Deactivating WidgetModule");
  // Deactivate publishers, timers, etc.
  if (widget_value_publisher_) {
        widget_value_publisher_->on_deactivate();
        RCLCPP_INFO(get_logger(), "Widget value publisher deactivated.");
    }
  return CallbackReturn::SUCCESS;
}

WidgetModule::CallbackReturn
WidgetModule::on_shutdown(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Shutting down WidgetModule");
  deinit();
  return CallbackReturn::SUCCESS;
}

bool WidgetModule::init()
{
    std::unique_lock<std::shared_mutex> lock(global_ptr_mutex_);
    if (is_module_initialized_)
    {
        RCLCPP_WARN(get_logger(), "WidgetModule already initialized, skipping.");
        return false;
    }

    RCLCPP_INFO(get_logger(), "Initiating Widgets");

    RCLCPP_INFO(get_logger(), widget_config_file_path_.c_str());

    widget_value_list_.resize(7, 0);
    widget_handler_list_ = {
        {0, DJI_WIDGET_TYPE_SWITCH, StaticSetWidgetValue, StaticGetWidgetValue, this},
        {1, DJI_WIDGET_TYPE_SWITCH, StaticSetWidgetValue, StaticGetWidgetValue, this},
        {2, DJI_WIDGET_TYPE_SWITCH, StaticSetWidgetValue, StaticGetWidgetValue, this},
        {3, DJI_WIDGET_TYPE_SWITCH, StaticSetWidgetValue, StaticGetWidgetValue, this},
        {4, DJI_WIDGET_TYPE_SWITCH, StaticSetWidgetValue, StaticGetWidgetValue, this},
        {5, DJI_WIDGET_TYPE_SWITCH, StaticSetWidgetValue, StaticGetWidgetValue, this},
        {6, DJI_WIDGET_TYPE_SWITCH, StaticSetWidgetValue, StaticGetWidgetValue, this},
    };

    T_DjiReturnCode return_code;

    return_code = DjiWidget_Init();
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Dji widget init error. Error code:  %ld", return_code);
        return false;
    }

    return_code = DjiWidget_RegDefaultUiConfigByDirPath(widget_config_file_path_.c_str());
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Add default widget ui config error. Error code:  %ld", return_code);
        return false;
    }

    return_code = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_ENGLISH, DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN, widget_config_file_path_.c_str());
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Add widget ui config error. Error code:  %ld", return_code);
        return false;
    }

    // return_code = DjiWidget_RegHandlerList(s_widgetHandlerList, s_widgetHandlerListCount);
    return_code = DjiWidget_RegHandlerList(widget_handler_list_.data(), widget_handler_list_.size());
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Set widget handler list error. Error code:  %ld", return_code);
        return false;
    }

    // Finalize Initialization
    is_module_initialized_ = true;
    RCLCPP_INFO(get_logger(), "WidgetModule initialized successfully with custom control configuration.");
    return true;
}

bool WidgetModule::deinit()
{
  std::unique_lock<std::shared_mutex> lock(global_ptr_mutex_);
  if (!is_module_initialized_)
  {
    RCLCPP_WARN(get_logger(), "WidgetModule is not initialized");
    return false;
  }

  // Clean up DJI SDK or widget-related resources here
  is_module_initialized_ = false;
  RCLCPP_INFO(get_logger(), "WidgetModule deinitialized successfully");
  return true;
}


T_DjiReturnCode WidgetModule::SetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value)
{
    RCLCPP_INFO(get_logger(), "Set widget value, widgetType = %d, widgetIndex = %d, widgetValue = %d", widgetType, index, value);

  widget_value_list_[index] = value;

  if (widget_value_publisher_)
  {
    // auto message = std_msgs::msg::UInt32();
    // message.data = value;

    psdk_interfaces::msg::WidgetPress msg;
    msg.widget_type = widgetType;
    msg.index = index;
    msg.value = value;
    widget_value_publisher_->publish(msg);
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode WidgetModule::GetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value)
{
  *value = widget_value_list_[index];
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode WidgetModule::StaticSetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value, void *userData)
{
  auto widget_module = static_cast<WidgetModule *>(userData);
  return widget_module->SetWidgetValue(widgetType, index, value);
}

T_DjiReturnCode WidgetModule::StaticGetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value, void *userData)
{
  auto widget_module = static_cast<WidgetModule *>(userData);
  return widget_module->GetWidgetValue(widgetType, index, value);
}

} // namespace psdk_ros2
