Compatible with ZED SDK 4.0 and ROS2 humble.

Simplified alternative to https://github.com/stereolabs/zed-ros2-wrapper with a focus on image acquisition only.

Other additional features:

* ros parameters for AEC_AGC_ROI (exposure ROI)
* timerless grab thread
* publisher for raw intrinsics (see below)
* (optional) publish device information, such as serial numbers, as transient_local topic

## raw intrinsics

If raw image publishing is enabled via `general.publish_raw`, the raw calibration parameters are published to `(left|right)/camera_info_raw`.
This is done to avoid incompatibilities with common ROS image tools, which often assume `camera_info` to be in the same namespace as an image (i.e. `image_rect`).
If you want to use the raw camera info in your image processing node, remap `camera_info_raw:=camera_info` and `camera_info:=camera_info_rect` in your instance of `zed_acquisition::ZedCamera`.