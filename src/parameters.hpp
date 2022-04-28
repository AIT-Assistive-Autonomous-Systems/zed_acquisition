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
  size_t field_start = prefix + 1;
  if (parameter.get_name().find("x", field_start) == field_start) {
    rect.x = parameter.get_value<double>();
  }
  if (parameter.get_name().find("y", field_start) == field_start) {
    rect.y = parameter.get_value<double>();
  }
  if (parameter.get_name().find("width", field_start) == field_start) {
    rect.width = parameter.get_value<double>();
  }
  if (parameter.get_name().find("height", field_start) == field_start) {
    rect.height = parameter.get_value<double>();
  }
}

} // namespace zed_acquisition
