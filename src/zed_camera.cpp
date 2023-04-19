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

#include <string>
#include <utility>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/camera_publisher.hpp>

#include <sl/Camera.hpp>
#include "parameters.hpp"
#include "ros_conversions.hpp"
#include "sl_tools.hpp"

namespace zed_acquisition
{
using namespace std::chrono_literals;
using namespace std::placeholders;
using diagnostic_msgs::msg::DiagnosticArray;

class ZedCamera : public rclcpp::Node
{
private:
  mutable std::mutex mutex_zed_close_;
  std::atomic_bool thread_stop_;

  std::string left_camera_optical_frame_;
  std::string right_camera_optical_frame_;
  std::string imu_frame_;
  std::string barometer_frame_;
  std::string magnetometer_frame_;

  sl::Camera zed_;
  diagnostic_updater::Updater diagnostic_updater_;
  std::thread grab_thread_;

  image_transport::CameraPublisher pub_left_image_rect_color_;
  image_transport::CameraPublisher pub_right_image_rect_color_;

  rclcpp::Publisher<Image>::SharedPtr pub_left_image_raw_color_;
  rclcpp::Publisher<Image>::SharedPtr pub_right_image_raw_color_;
  rclcpp::Publisher<CameraInfo>::SharedPtr pub_left_camera_info_raw_;
  rclcpp::Publisher<CameraInfo>::SharedPtr pub_right_camera_info_raw_;

  image_transport::Publisher pub_left_depth_;
  image_transport::Publisher pub_left_disparity_;
  // extra camera info topic for when depth is published at a different resolution
  rclcpp::Publisher<CameraInfo>::SharedPtr pub_depth_camera_info_;

  rclcpp::Publisher<Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<FluidPressure>::SharedPtr pub_atm_press_;
  rclcpp::Publisher<MagneticField>::SharedPtr pub_imu_mag_;

  rclcpp::Publisher<Temperature>::SharedPtr pub_imu_temp_;
  rclcpp::Publisher<Temperature>::SharedPtr pub_left_temp_;
  rclcpp::Publisher<Temperature>::SharedPtr pub_right_temp_;

  rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_device_information_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_parameters_;
  rclcpp::TimerBase::SharedPtr timer_sensor_publish_;

  sl::Rect aec_agc_roi_left_;
  sl::Rect aec_agc_roi_right_;

public:
  explicit ZedCamera(const rclcpp::NodeOptions & options)
  : Node("zed_camera", options), diagnostic_updater_(this)
  {
    RCLCPP_INFO(
      get_logger(), "SDK Version: %d.%d.%d - Build %s",
      ZED_SDK_MAJOR_VERSION,
      ZED_SDK_MINOR_VERSION,
      ZED_SDK_PATCH_VERSION,
      ZED_SDK_BUILD_ID);
    if (ZED_SDK_MAJOR_VERSION < 3 || (ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 5)) {
      RCLCPP_ERROR(
        get_logger(),
        "ZedCamera is designed to work with ZED SDK v3.3 or newer.");
      exit(EXIT_FAILURE);
    }

    auto camera_id = declare_parameter("general.camera_id", 0);
    auto serial_number = declare_parameter("general.serial_number", 0);

    declare_parameter("general.publish_every", 1);

    sl::InitParameters init_params;
    init_params.camera_fps = declare_parameter("general.grab_frame_rate", 0);
    init_params.camera_resolution =
      static_cast<sl::RESOLUTION>(declare_parameter("general.resolution", 0));

    if (serial_number == 0) {
      init_params.input.setFromCameraID(camera_id);
    } else {
      init_params.input.setFromSerialNumber(serial_number);
    }

    declare_parameter("depth.enable_fill_mode", false);
    declare_parameter("depth.resolution.width", 0);
    declare_parameter("depth.resolution.height", 0);
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.depth_mode = static_cast<sl::DEPTH_MODE>(declare_parameter("depth.depth_mode", 0));
    init_params.sdk_verbose = declare_parameter("general.sdk_verbose", false);

    int gpu_id = declare_parameter("general.gpu_id", 0);
    init_params.sdk_gpu_id = gpu_id;

    cuCtxGetDevice(&gpu_id);
    RCLCPP_INFO_STREAM(get_logger(), "ZED SDK running on GPU #" << gpu_id);

    init_params.depth_stabilization =
      static_cast<int>(declare_parameter("depth.depth_stabilization", false));
    init_params.depth_minimum_distance = declare_parameter("depth.minimum_distance", 0.2);
    init_params.depth_maximum_distance = declare_parameter("depth.maximum_distance", 10.0);

    auto calibration_file = declare_parameter("general.optional_opencv_calibration_file", "");
    if (!calibration_file.empty()) {
      init_params.optional_opencv_calibration_file = calibration_file.c_str();
    }
    init_params.camera_disable_self_calib = declare_parameter(
      "general.camera_disable_self_calib",
      false);
    init_params.enable_image_enhancement = declare_parameter(
      "general.enable_image_enhancement",
      true);
    init_params.open_timeout_sec = declare_parameter(
      "general.open_timeout_sec",
      5.0);
    init_params.enable_right_side_measure = false;

    if (!startCamera(init_params)) {
      exit(EXIT_FAILURE);
    }

    auto camera_information = zed_.getCameraInformation();
    auto camera_name =
      declare_parameter("general.camera_name", toString(camera_information.camera_model));
    auto hardware_id = "zed" + std::to_string(camera_information.serial_number);

    left_camera_optical_frame_ = declare_parameter(
      "left_camera_optical_frame",
      camera_name + "_left_camera_optical_frame");

    right_camera_optical_frame_ = declare_parameter(
      "right_camera_optical_frame",
      camera_name + "_right_camera_optical_frame");

    imu_frame_ = declare_parameter("imu_frame", camera_name + "_imu_link");
    barometer_frame_ = declare_parameter("barometer_frame", camera_name + "_left_camera_frame");
    magnetometer_frame_ = declare_parameter("magnetometer_frame", camera_name + "_imu_link");

    diagnostic_updater_.add("ZED Diagnostic", this, &ZedCamera::callback_diagnostic);
    diagnostic_updater_.setHardwareID(hardware_id);

    // declare roi params before setting callback, to set them atomically later
    declare_parameters<int>(
      "video.aec_agc_roi.left", {
        {"x", 0},
        {"y", 0},
        {"width", 0},
        {"height", 0}});
    declare_parameters<int>(
      "video.aec_agc_roi.right", {
        {"x", 0},
        {"y", 0},
        {"width", 0},
        {"height", 0}});

    callback_parameters_ =
      add_on_set_parameters_callback(std::bind(&ZedCamera::parameter_callback, this, _1));

    declare_parameter("video.resolution.width", 0);
    declare_parameter("video.resolution.height", 0);

    declare_integer_range(get_node_parameters_interface(), "video.brightness", 4, 0, 8);
    declare_integer_range(get_node_parameters_interface(), "video.contrast", 4, 0, 8);
    declare_integer_range(get_node_parameters_interface(), "video.sharpness", 4, 0, 8);
    declare_integer_range(get_node_parameters_interface(), "video.hue", 0, 0, 8);
    declare_integer_range(get_node_parameters_interface(), "video.saturation", 4, 0, 8);
    declare_integer_range(get_node_parameters_interface(), "video.gamma", 8, 0, 8);
    declare_integer_range(get_node_parameters_interface(), "video.exposure", 80, 0, 100);
    declare_integer_range(get_node_parameters_interface(), "video.gain", 80, 0, 100);
    declare_parameter("video.aec_agc", true);

    // trigger roi initialization after declaring them, to only set the rect once
    std::map<std::string, rclcpp::Parameter> roi_param_map;
    get_parameters("video.aec_agc_roi", roi_param_map);
    std::vector<rclcpp::Parameter> roi_params;
    roi_params.reserve(roi_param_map.size());
    std::transform(
      roi_param_map.begin(), roi_param_map.end(), std::back_inserter(
        roi_params), [](const auto & item) {
        return item.second;
      });
    parameter_callback(roi_params);

    declare_parameter("video.auto_whitebalance", true);

    pub_left_image_rect_color_ = image_transport::create_camera_publisher(
      this, "left/image_rect_color", rmw_qos_profile_sensor_data);

    pub_right_image_rect_color_ = image_transport::create_camera_publisher(
      this, "right/image_rect_color", rmw_qos_profile_sensor_data);

    if (declare_parameter("general.publish_raw", false)) {
      pub_left_image_raw_color_ = create_publisher<Image>(
        "left/image_raw_color",
        rclcpp::SensorDataQoS());
      pub_right_image_raw_color_ = create_publisher<Image>(
        "right/image_raw_color",
        rclcpp::SensorDataQoS());

      pub_left_camera_info_raw_ = create_publisher<CameraInfo>(
        "left/camera_info_raw",
        rclcpp::SensorDataQoS());
      pub_right_camera_info_raw_ = create_publisher<CameraInfo>(
        "right/camera_info_raw",
        rclcpp::SensorDataQoS());
    }

    if (zed_.getInitParameters().depth_mode != sl::DEPTH_MODE::NONE) {
      if (declare_parameter("general.publish_depth", true)) {
        pub_left_depth_ = image_transport::create_publisher(
          this, "left/depth", rmw_qos_profile_sensor_data);
      }
      if (declare_parameter("general.publish_disparity", false)) {
        pub_left_disparity_ = image_transport::create_publisher(
          this, "left/disparity", rmw_qos_profile_sensor_data);
      }
    }

    // publish information on connected camera once
    auto camera_information_qos = rclcpp::QoS(1).transient_local().reliable();
    pub_device_information_ = create_publisher<DiagnosticArray>(
      "device_information", camera_information_qos);

    DiagnosticArray device_information;
    device_information.header.frame_id = left_camera_optical_frame_;
    device_information.header.stamp = now();
    device_information.status.resize(1);
    device_information.status[0].name = "Camera information";
    device_information.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    device_information.status[0].message = "ZED connected";
    device_information.status[0].hardware_id = hardware_id;
    device_information.status[0].values.resize(4);
    device_information.status[0].values[0].key = "Serial number";
    device_information.status[0].values[0].value = std::to_string(
      camera_information.serial_number);
    device_information.status[0].values[1].key = "Model";
    device_information.status[0].values[1].value = toString(camera_information.camera_model);
    device_information.status[0].values[2].key = "Camera firmware version";
    device_information.status[0].values[2].value = std::to_string(
      camera_information.camera_configuration.firmware_version);

    device_information.status[0].values[3].key = "Sensors firmware version";
    device_information.status[0].values[3].value = std::to_string(
      camera_information.sensors_configuration.firmware_version);

    pub_device_information_->publish(device_information);

    if (camera_information.camera_model != sl::MODEL::ZED) {
      pub_imu_ = create_publisher<Imu>("imu/data", rclcpp::SensorDataQoS());
      pub_imu_temp_ = create_publisher<Temperature>(
        "imu/temperature",
        rclcpp::SensorDataQoS());

      pub_imu_mag_ = create_publisher<MagneticField>("imu/mag", rclcpp::SensorDataQoS());
      pub_atm_press_ = create_publisher<FluidPressure>("atm_press", rclcpp::SensorDataQoS());

      pub_left_temp_ = create_publisher<Temperature>("left/temperature", rclcpp::SensorDataQoS());
      pub_right_temp_ = create_publisher<Temperature>("right/temperature", rclcpp::SensorDataQoS());

      auto rate = declare_parameter("sensors.publish_rate", 400.0);
      timer_sensor_publish_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&ZedCamera::callback_publish_sensors, this));
    }
    grab_thread_ = std::thread(&ZedCamera::run_grab, this);
  }

  ~ZedCamera()
  {
    std::lock_guard<std::mutex> lock(mutex_zed_close_);
    thread_stop_ = true;
    RCLCPP_DEBUG(get_logger(), "Stopping grab thread");
    try {
      if (grab_thread_.joinable()) {
        grab_thread_.join();
      }
    } catch (std::system_error & e) {
      RCLCPP_WARN(get_logger(), "Grab thread joining exception: %s", e.what());
    }
    RCLCPP_DEBUG(get_logger(), "Grab thread stopped");
  }

  rcl_interfaces::msg::SetParametersResult
  parameter_callback(std::vector<rclcpp::Parameter> parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool roi_left_dirty = false;
    bool roi_right_dirty = false;

    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "video.aec_agc") {
        RCLCPP_INFO(get_logger(), "Set AEC_AGC %i", parameter.as_bool());
        zed_.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, parameter.as_bool());
      }
      if (parameter.get_name() == "video.gain") {
        RCLCPP_INFO(get_logger(), "Set GAIN %li", parameter.as_int());
        zed_.setCameraSettings(sl::VIDEO_SETTINGS::GAIN, parameter.as_int());
      }
      if (parameter.get_name() == "video.exposure") {
        RCLCPP_INFO(get_logger(), "Set EXPOSURE %li", parameter.as_int());
        zed_.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, parameter.as_int());
      }
      if (parameter.get_name() == "video.brightness") {
        RCLCPP_INFO(get_logger(), "Set BRIGHTNESS %li", parameter.as_int());
        zed_.setCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS, parameter.as_int());
      }
      if (parameter.get_name() == "video.whitebalance_auto") {
        RCLCPP_INFO(get_logger(), "Set WHITEBALANCE_AUTO %li", parameter.as_int());
        zed_.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, parameter.as_int());
      }
      if (parameter.get_name() == "video.whitebalance_temperature") {
        RCLCPP_INFO(get_logger(), "Set WHITEBALANCE_TEMPERATURE %li", parameter.as_int());
        zed_.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, parameter.as_int());
      }
      if (parameter.get_name() == "video.gamma") {
        RCLCPP_INFO(get_logger(), "Set GAMMA %li", parameter.as_int());
        zed_.setCameraSettings(sl::VIDEO_SETTINGS::GAMMA, parameter.as_int());
      }
      if (parameter.get_name() == "video.sharpness") {
        RCLCPP_INFO(get_logger(), "Set SHARPNESS %li", parameter.as_int());
        zed_.setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, parameter.as_int());
      }
      if (parameter.get_name() == "video.hue") {
        RCLCPP_INFO(get_logger(), "Set HUE %li", parameter.as_int());
        zed_.setCameraSettings(sl::VIDEO_SETTINGS::HUE, parameter.as_int());
      }
      if (parameter.get_name() == "video.saturation") {
        RCLCPP_INFO(get_logger(), "Set SATURATION %li", parameter.as_int());
        zed_.setCameraSettings(sl::VIDEO_SETTINGS::SATURATION, parameter.as_int());
      }
      if (parameter.get_name() == "video.contrast") {
        RCLCPP_INFO(get_logger(), "Set CONTRAST %li", parameter.as_int());
        zed_.setCameraSettings(sl::VIDEO_SETTINGS::CONTRAST, parameter.as_int());
      }

      static constexpr char left_rect_prefix[] = "video.aec_agc_roi.left.";
      if (parameter.get_name().find(left_rect_prefix) == 0) {
        set_rect_parameter(parameter, sizeof(left_rect_prefix) - 1, aec_agc_roi_left_);
        roi_left_dirty = true;
      }
      static constexpr char right_rect_prefix[] = "video.aec_agc_roi.right.";
      if (parameter.get_name().find(right_rect_prefix) == 0) {
        set_rect_parameter(parameter, sizeof(right_rect_prefix) - 1, aec_agc_roi_right_);
        roi_right_dirty = true;
      }
    }

    if (roi_left_dirty) {
      RCLCPP_INFO(
        get_logger(), "Set AEC_AGX_ROI left to %lux%lu@(x=%lu,y=%lu)", aec_agc_roi_left_.width, aec_agc_roi_left_.height, aec_agc_roi_left_.x,
        aec_agc_roi_left_.y);
      zed_.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC_ROI, aec_agc_roi_left_, sl::SIDE::LEFT);
    }
    if (roi_right_dirty) {
      RCLCPP_INFO(
        get_logger(), "Set AEC_AGX_ROI right to %lux%lu@(x=%lu,y=%lu)", aec_agc_roi_right_.width, aec_agc_roi_right_.height, aec_agc_roi_right_.x,
        aec_agc_roi_right_.y);
      zed_.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC_ROI, aec_agc_roi_right_, sl::SIDE::RIGHT);
    }
    return result;
  }

  bool startCamera(sl::InitParameters init_params)
  {
    RCLCPP_DEBUG(get_logger(), "Camera opening");

    thread_stop_ = false;

    auto connection_status = zed_.open(init_params);
    if (connection_status == sl::ERROR_CODE::SUCCESS) {
      RCLCPP_INFO(get_logger(), "Camera opened");
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "Error opening camera: %s",
        sl::toString(connection_status).c_str());
      return false;
    }

    if (!rclcpp::ok() || thread_stop_) {
      RCLCPP_INFO(get_logger(), "ZED activation interrupted");
      return false;
    }

    const auto & camera_info = zed_.getCameraInformation();
    RCLCPP_INFO_STREAM(
      get_logger(), "Camera model '" << sl::toString(
        camera_info.camera_model) << "'");
    RCLCPP_INFO_STREAM(get_logger(), "Serial number '" << camera_info.serial_number << "'");

    RCLCPP_INFO(
      get_logger(), "Camera FW version '%u'",
      camera_info.camera_configuration.firmware_version);
    if (camera_info.camera_model != sl::MODEL::ZED) {
      auto sensor_fw_version = camera_info.sensors_configuration.firmware_version;
      RCLCPP_INFO_STREAM(
        get_logger(),
        "Sensors FW Version '" << sensor_fw_version << "'");
    }
    return true;
  }

  void run_grab()
  {
    RCLCPP_DEBUG(get_logger(), "run_grab started");
    sl::RuntimeParameters run_params;
    run_params.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
    run_params.remove_saturated_areas = false;

    while (1) {
      if (!rclcpp::ok()) {
        break;
      }
      static int frame_count = -1;
      frame_count = (frame_count + 1) % get_parameter("general.publish_every").as_int();
      bool publish = frame_count == 0;
      run_params.enable_fill_mode = get_parameter("depth.enable_fill_mode").as_bool();
      run_params.enable_depth = publish &&
        (zed_.getInitParameters().depth_mode != sl::DEPTH_MODE::NONE);
      auto grab_status = zed_.grab(run_params);
      if (grab_status != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Camera error: " << sl::toString(grab_status));
        break;
      }
      if (publish) {
        retrieve_and_publish();
      }
    }
    RCLCPP_DEBUG(get_logger(), "run_grab finished");
  }

  void callback_publish_sensors()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_zed_close_);
      if (!zed_.isOpened()) {
        return;
      }
    }
    sl::SensorsData sensor_data;
    if (zed_.getSensorsData(sensor_data, sl::TIME_REFERENCE::CURRENT) != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to get sensors data");
      return;
    }

    auto imu = toMsg(sensor_data.imu);
    auto t = imu->header.stamp;

    if (pub_imu_) {
      auto imu = toMsg(sensor_data.imu);
      imu->header.frame_id = imu_frame_;
      pub_imu_->publish(std::move(imu));
    }

    if (pub_imu_temp_) {
      float imu_temp;
      auto result = sensor_data.temperature.get(
        sl::SensorsData::TemperatureData::SENSOR_LOCATION::IMU, imu_temp);
      if (result == sl::ERROR_CODE::SUCCESS) {
        auto temp = std::make_unique<Temperature>();
        temp->header.stamp = t;
        temp->header.frame_id = imu_frame_;
        temp->temperature = static_cast<double>(imu_temp);
        temp->variance = 0.0;
        pub_imu_temp_->publish(std::move(temp));
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to get IMU temperature");
      }
    }

    if (sensor_data.barometer.is_available && pub_atm_press_) {
      auto pressure = toMsg(sensor_data.barometer);
      pressure->header.frame_id = barometer_frame_;
      pub_atm_press_->publish(std::move(pressure));
    }

    if (sensor_data.magnetometer.is_available && pub_imu_mag_) {
      auto msg = toMsg(sensor_data.magnetometer);
      msg->header.frame_id = magnetometer_frame_;
      pub_imu_mag_->publish(std::move(msg));
    }

    if (pub_left_temp_) {
      float left_temp;
      sensor_data.temperature.get(
        sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_LEFT,
        left_temp);

      auto temp = std::make_unique<Temperature>();
      temp->header.stamp = t;
      temp->header.frame_id = left_camera_optical_frame_;
      temp->temperature = static_cast<double>(left_temp);
      temp->variance = 0.0;
      pub_left_temp_->publish(std::move(temp));
    }

    if (pub_right_temp_) {
      float right_temp;
      sensor_data.temperature.get(
        sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_RIGHT,
        right_temp);

      auto temp = std::make_unique<Temperature>();
      temp->header.stamp = t;
      temp->header.frame_id = right_camera_optical_frame_;
      temp->temperature = static_cast<double>(right_temp);
      temp->variance = 0.0;
      pub_right_temp_->publish(std::move(temp));
    }
  }

  void retrieve_and_publish()
  {
    auto resolution = sl::Resolution(
      get_parameter("video.resolution.width").as_int(),
      get_parameter("video.resolution.height").as_int());
    auto camera_info = zed_.getCameraInformation(resolution);

    sl::ERROR_CODE retrieve_result;

    sl::Mat mat_left_rect;
    retrieve_result = zed_.retrieveImage(
      mat_left_rect, sl::VIEW::LEFT, sl::MEM::CPU, resolution);
    if (retrieve_result != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Retrieve left failed");
      return;
    }

    sl::Mat mat_right_rect;
    retrieve_result = zed_.retrieveImage(
      mat_right_rect, sl::VIEW::RIGHT, sl::MEM::CPU, resolution);
    if (retrieve_result != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Retrieve right failed");
      return;
    }

    auto depth_width = get_parameter("depth.resolution.width").as_int();
    auto depth_height = get_parameter("depth.resolution.height").as_int();
    sl::Resolution depth_resolution(depth_width, depth_height);

    sl::Mat mat_depth;
    if (pub_left_depth_) {
      retrieve_result = zed_.retrieveMeasure(
        mat_depth, sl::MEASURE::DEPTH, sl::MEM::CPU, depth_resolution);
      if (retrieve_result != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Retrieve depth failed");
        return;
      }
      if (mat_depth.getDataType() != sl::MAT_TYPE::F32_C1) {
        RCLCPP_ERROR(get_logger(), "depth image is not float32");
        return;
      }
    }

    sl::Mat mat_disparity;
    if (pub_left_disparity_) {
      retrieve_result = zed_.retrieveMeasure(
        mat_disparity, sl::MEASURE::DISPARITY, sl::MEM::CPU, depth_resolution);
      if (retrieve_result != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Retrieve depth failed");
        return;
      }
      if (mat_disparity.getDataType() != sl::MAT_TYPE::F32_C1) {
        RCLCPP_ERROR(get_logger(), "disparity image is not float32");
        return;
      }
    }

    sl::Mat mat_left_raw;
    if (pub_left_image_raw_color_) {
      rcpputils::assert_true(!!pub_left_camera_info_raw_);
      retrieve_result = zed_.retrieveImage(
        mat_left_raw, sl::VIEW::LEFT_UNRECTIFIED, sl::MEM::CPU, resolution);
      if (retrieve_result != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Retrieve left raw failed");
        return;
      }
    }

    sl::Mat mat_right_raw;
    if (pub_right_image_raw_color_) {
      rcpputils::assert_true(!!pub_right_camera_info_raw_);
      retrieve_result = zed_.retrieveImage(
        mat_right_raw, sl::VIEW::RIGHT_UNRECTIFIED, sl::MEM::CPU, resolution);
      if (retrieve_result != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Retrieve right raw failed");
        return;
      }
    }

    auto calibration_params = camera_info.camera_configuration.calibration_parameters;

    auto left_ts = rclcpp::Time(
      mat_left_rect.timestamp.getNanoseconds(),
      get_clock()->get_clock_type());
    auto left_rect_msg = toMsg(mat_left_rect);
    left_rect_msg->header.frame_id = left_camera_optical_frame_;
    left_rect_msg->header.stamp = left_ts;
    auto left_camera_info = getCameraInfo(calibration_params, sl::SIDE::LEFT);
    left_camera_info->header = left_rect_msg->header;
    pub_left_image_rect_color_.publish(left_rect_msg, left_camera_info);

    auto right_ts =
      rclcpp::Time(mat_right_rect.timestamp.getNanoseconds(), get_clock()->get_clock_type());
    auto right_rect_msg = toMsg(mat_right_rect);
    right_rect_msg->header.frame_id = right_camera_optical_frame_;
    right_rect_msg->header.stamp = right_ts;
    auto right_camera_info = getCameraInfo(calibration_params, sl::SIDE::RIGHT);
    right_camera_info->header = right_rect_msg->header;
    pub_right_image_rect_color_.publish(right_rect_msg, right_camera_info);

    if (pub_left_depth_) {
      auto depth_ts =
        rclcpp::Time(mat_depth.timestamp.getNanoseconds(), get_clock()->get_clock_type());
      auto depth_msg = toMsg(mat_depth);
      depth_msg->header.frame_id = left_camera_optical_frame_;
      depth_msg->header.stamp = depth_ts;
      pub_left_depth_.publish(depth_msg);
    }

    if (pub_left_disparity_) {
      auto disparity_ts =
        rclcpp::Time(mat_disparity.timestamp.getNanoseconds(), get_clock()->get_clock_type());
      auto disparity_msg = toMsg(mat_disparity);
      disparity_msg->header.frame_id = left_camera_optical_frame_;
      disparity_msg->header.stamp = disparity_ts;
      pub_left_disparity_.publish(disparity_msg);
    }


    if (zed_.getInitParameters().depth_mode != sl::DEPTH_MODE::NONE) {
      if (depth_resolution.width != 0 || depth_resolution.height != 0) {
        // when depth is enabled, but a resolution different from video is configured
        // publish the camera info for the other resolution in a different namespace
        // when properly remapped, this helps keep the convention of having the camera_info in the
        // same namespace as the image define its intrinsics
        if (!pub_depth_camera_info_) {
          pub_depth_camera_info_ = create_publisher<CameraInfo>(
            "left/depth/camera_info",
            rclcpp::SensorDataQoS());
        }
        if (pub_depth_camera_info_) {
          auto depth_camera_info = getCameraInfo(
            zed_.getCameraInformation(
              depth_resolution).camera_configuration.calibration_parameters, sl::SIDE::LEFT);
          depth_camera_info->header = left_rect_msg->header;
          pub_depth_camera_info_->publish(*depth_camera_info);
        }
      }
    }

    auto calibration_params_raw = camera_info.camera_configuration.calibration_parameters_raw;
    if (pub_left_image_raw_color_) {
      auto left_raw_msg = toMsg(mat_left_raw);
      left_raw_msg->header = left_rect_msg->header;
      auto left_camera_info_raw = getCameraInfo(calibration_params_raw, sl::SIDE::LEFT);
      left_camera_info_raw->header = left_raw_msg->header;
      pub_left_image_raw_color_->publish(*left_raw_msg);
      pub_left_camera_info_raw_->publish(*left_camera_info_raw);
    }
    if (pub_right_image_raw_color_) {
      auto right_raw_msg = toMsg(mat_right_raw);
      right_raw_msg->header = right_rect_msg->header;
      auto right_camera_info_raw = getCameraInfo(calibration_params_raw, sl::SIDE::RIGHT);
      right_camera_info_raw->header = right_raw_msg->header;
      pub_right_image_raw_color_->publish(*right_raw_msg);
      pub_right_camera_info_raw_->publish(*right_camera_info_raw);
    }
  }

  void callback_diagnostic(
    diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (!zed_.isOpened()) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Camera Closed");
      return;
    }
    stat.add("Left CMOS Temp.", "N/A");
    stat.add("Right CMOS Temp.", "N/A");
  }
};

} // namespace zed_acquisition

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(zed_acquisition::ZedCamera)
