cmake_minimum_required(VERSION 3.8)
project(zed_acquisition)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ZED 3 REQUIRED)

exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" ) # Jetson platforms
    SET(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
endif()

find_package(CUDA REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(cv_bridge REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/zed_camera.cpp
  src/image_encode.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PRIVATE
  src
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  "angles"
  "rclcpp"
  "rclcpp_components"
  "builtin_interfaces"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "image_transport"
  "diagnostic_msgs"
  "diagnostic_updater"
  "cv_bridge"
)

rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "zed_acquisition::ZedCamera" EXECUTABLE zed_camera_node)
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "zed_acquisition::ImageEncode" EXECUTABLE image_encode_node)
set_target_properties(zed_camera_node PROPERTIES OUTPUT_NAME "zed_camera")
set_target_properties(image_encode_node PROPERTIES OUTPUT_NAME "image_encode")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${PROJECT_NAME})
ament_package()
