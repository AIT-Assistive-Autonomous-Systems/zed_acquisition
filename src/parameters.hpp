// Copyright 2022 AIT Austrian Institute of Technology GmbH
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
  parameters->declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor);
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
