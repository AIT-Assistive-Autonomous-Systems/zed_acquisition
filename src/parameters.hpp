#pragma once

#include <string>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <sl/Camera.hpp>

namespace zed_acquisition {

using rcl_interfaces::msg::ParameterDescriptor;
using rcl_interfaces::msg::IntegerRange;
using rclcpp::node_interfaces::NodeParametersInterface;

void declare_integer_range(
  const NodeParametersInterface::SharedPtr & parameters,
  const std::string & name,
  int default_value,
  int low, int high)
{
  ParameterDescriptor descriptor;
  descriptor.name = name;
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = low;
  range.to_value = high;
  range.step = 1;
  descriptor.integer_range.push_back(range);
  parameters->declare_parameter(name, rclcpp::ParameterValue(4), descriptor);
}

void set_rect_parameter(const rclcpp::Parameter& parameter, size_t prefix, sl::Rect& rect) {
  if (parameter.get_name().find("x", prefix) == prefix) {
    rect.x = parameter.get_value<int>();
  }
  if (parameter.get_name().find("y", prefix) == prefix) {
    rect.y = parameter.get_value<int>();
  }
  if (parameter.get_name().find("width", prefix) == prefix) {
    rect.width = parameter.get_value<int>();
  }
  if (parameter.get_name().find("height", prefix) == prefix) {
    rect.height = parameter.get_value<int>();
  }
}

} // namespace zed_acquisition
