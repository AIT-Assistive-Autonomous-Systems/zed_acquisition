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
#include <vector>
#include <memory>
#include <algorithm>
#include <angles/angles.h>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <sl/Camera.hpp>

namespace zed_acquisition
{

const double DEG_TO_RAD_FACTOR = 0.017453293;

using sensor_msgs::msg::Image;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::MagneticField;
using sensor_msgs::msg::Temperature;
using sensor_msgs::msg::FluidPressure;

CameraInfo::SharedPtr toMsg(const sl::CameraParameters & camera_params)
{
  // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  auto msg = std::make_shared<CameraInfo>();
  msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  msg->d.resize(5);
  msg->d[0] = camera_params.disto[0]; // k1
  msg->d[1] = camera_params.disto[1]; // k2
  msg->d[2] = camera_params.disto[4]; // k3
  msg->d[3] = camera_params.disto[2]; // p1
  msg->d[4] = camera_params.disto[3]; // p2
  msg->k.fill(0.0);
  msg->k[0] = static_cast<double>(camera_params.fx);
  msg->k[2] = static_cast<double>(camera_params.cx);
  msg->k[4] = static_cast<double>(camera_params.fy);
  msg->k[5] = static_cast<double>(camera_params.cy);
  msg->k[8] = 1.0;
  msg->r.fill(0.0);

  // set rectified identity
  for (size_t i = 0; i < 3; i++) {
    msg->r[i + i * 3] = 1;
  }

  msg->p.fill(0.0);
  msg->p[0] = static_cast<double>(camera_params.fx);
  msg->p[2] = static_cast<double>(camera_params.cx);
  msg->p[5] = static_cast<double>(camera_params.fy);
  msg->p[6] = static_cast<double>(camera_params.cy);
  msg->p[10] = 1.0;
  msg->width = static_cast<uint32_t>(camera_params.image_size.width);
  msg->height = static_cast<uint32_t>(camera_params.image_size.height);
  return msg;
}

std::shared_ptr<CameraInfo> getCameraInfo(
  sl::CalibrationParameters calibration_params,
  sl::SIDE side)
{
  if (side == sl::SIDE::LEFT) {
    return toMsg(calibration_params.left_cam);
  } else if (side == sl::SIDE::RIGHT) {
    auto msg = toMsg(calibration_params.right_cam);
    auto baseline = calibration_params.getCameraBaseline();
    auto matrix = calibration_params.stereo_transform.getRotationMatrix();
    matrix.inverse();
    std::copy(std::begin(matrix.r), std::end(matrix.r), std::begin(msg->r));
    msg->p[3] = static_cast<double>(-1 * calibration_params.right_cam.fx * baseline);
    return msg;
  } else {
    throw std::invalid_argument("BOTH is an invalid side for getCameraInfo");
  }
}

Imu::UniquePtr toMsg(sl::SensorsData::IMUData & imu)
{
  auto msg = std::make_unique<Imu>();
  msg->header.stamp = rclcpp::Time(imu.timestamp.getNanoseconds(), RCL_ROS_TIME);

  msg->orientation.x = imu.pose.getOrientation()[0];
  msg->orientation.y = imu.pose.getOrientation()[1];
  msg->orientation.z = imu.pose.getOrientation()[2];
  msg->orientation.w = imu.pose.getOrientation()[3];

  msg->angular_velocity.x = angles::from_degrees(imu.angular_velocity[0]);
  msg->angular_velocity.y = angles::from_degrees(imu.angular_velocity[1]);
  msg->angular_velocity.z = angles::from_degrees(imu.angular_velocity[2]);

  msg->linear_acceleration.x = imu.linear_acceleration[0];
  msg->linear_acceleration.y = imu.linear_acceleration[1];
  msg->linear_acceleration.z = imu.linear_acceleration[2];

  // NOTE: memcpy not allowed because ROS2 uses double and ZED SDK uses float
  for (int i = 0; i < 3; ++i) {
    int r = 0;
    if (i == 0) {
      r = 0;
    } else if (i == 1) {
      r = 1;
    } else {
      r = 2;
    }

    msg->orientation_covariance[i * 3 + 0] = imu.pose_covariance.r[r * 3 + 0] *
      DEG_TO_RAD_FACTOR * DEG_TO_RAD_FACTOR;
    msg->orientation_covariance[i * 3 + 1] = imu.pose_covariance.r[r * 3 + 1] *
      DEG_TO_RAD_FACTOR * DEG_TO_RAD_FACTOR;
    msg->orientation_covariance[i * 3 + 2] = imu.pose_covariance.r[r * 3 + 2] *
      DEG_TO_RAD_FACTOR * DEG_TO_RAD_FACTOR;

    msg->linear_acceleration_covariance[i * 3 +
      0] = imu.linear_acceleration_covariance.r[r * 3 + 0];
    msg->linear_acceleration_covariance[i * 3 +
      1] = imu.linear_acceleration_covariance.r[r * 3 + 1];
    msg->linear_acceleration_covariance[i * 3 +
      2] = imu.linear_acceleration_covariance.r[r * 3 + 2];

    msg->angular_velocity_covariance[i * 3 +
      0] = imu.angular_velocity_covariance.r[r * 3 + 0] * DEG_TO_RAD_FACTOR * DEG_TO_RAD_FACTOR;
    msg->angular_velocity_covariance[i * 3 +
      1] = imu.angular_velocity_covariance.r[r * 3 + 1] * DEG_TO_RAD_FACTOR * DEG_TO_RAD_FACTOR;
    msg->angular_velocity_covariance[i * 3 +
      2] = imu.angular_velocity_covariance.r[r * 3 + 2] * DEG_TO_RAD_FACTOR * DEG_TO_RAD_FACTOR;
  }
  return msg;
}

MagneticField::UniquePtr toMsg(sl::SensorsData::MagnetometerData & magnetometer)
{
  auto msg = std::make_unique<MagneticField>();
  auto t = rclcpp::Time(magnetometer.timestamp.getNanoseconds(), RCL_ROS_TIME);
  msg->header.stamp = t;
  msg->magnetic_field.x = magnetometer.magnetic_field_calibrated.x * 1e-6; // Tesla
  msg->magnetic_field.y = magnetometer.magnetic_field_calibrated.y * 1e-6; // Tesla
  msg->magnetic_field.z = magnetometer.magnetic_field_calibrated.z * 1e-6; // Tesla
  msg->magnetic_field_covariance[0] = 0.039e-6;
  msg->magnetic_field_covariance[1] = 0.0f;
  msg->magnetic_field_covariance[2] = 0.0f;
  msg->magnetic_field_covariance[3] = 0.0f;
  msg->magnetic_field_covariance[4] = 0.037e-6;
  msg->magnetic_field_covariance[5] = 0.0f;
  msg->magnetic_field_covariance[6] = 0.0f;
  msg->magnetic_field_covariance[7] = 0.0f;
  msg->magnetic_field_covariance[8] = 0.047e-6;
  return msg;
}

FluidPressure::UniquePtr toMsg(sl::SensorsData::BarometerData & barometer)
{
  auto msg = std::make_unique<FluidPressure>();
  msg->header.stamp = rclcpp::Time(barometer.timestamp.getNanoseconds(), RCL_ROS_TIME);
  msg->fluid_pressure = barometer.pressure * 1e2; // Pascal
  msg->variance = 1.0585e-2;
  return msg;
}


Image::SharedPtr toMsg(sl::Mat & img)
{
  auto msg = std::make_shared<Image>();
  msg->height = img.getHeight();
  msg->width = img.getWidth();
  msg->step = img.getStepBytes();
  size_t size = msg->step * msg->height;
  uint8_t * data_ptr = nullptr;

  sl::MAT_TYPE dataType = img.getDataType();

  switch (dataType) {
    case sl::MAT_TYPE::F32_C1: /**< float 1 channel.*/
      msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float1>());
      break;

    case sl::MAT_TYPE::F32_C2: /**< float 2 channels.*/
      msg->encoding = sensor_msgs::image_encodings::TYPE_32FC2;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float2>());
      break;

    case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
      msg->encoding = sensor_msgs::image_encodings::TYPE_32FC3;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float3>());
      break;

    case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
      msg->encoding = sensor_msgs::image_encodings::TYPE_32FC4;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float4>());
      break;

    case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
      msg->encoding = sensor_msgs::image_encodings::MONO8;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar1>());
      break;

    case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
      msg->encoding = sensor_msgs::image_encodings::TYPE_8UC2;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar2>());
      break;

    case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
      msg->encoding = sensor_msgs::image_encodings::BGR8;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar3>());
      break;

    case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
      msg->encoding = sensor_msgs::image_encodings::BGRA8;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar4>());
      break;
  }
  msg->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
  return msg;
}

std::string toString(sl::MODEL & model)
{
  switch (model) {
    case sl::MODEL::ZED: return "zed";
    case sl::MODEL::ZED_M: return "zed_m";
    case sl::MODEL::ZED2: return "zed2";
    case sl::MODEL::ZED2i: return "zed2i";
    default: throw std::invalid_argument("unknown sl::MODEL");
  }
}

} // namespace zed_acquisition
