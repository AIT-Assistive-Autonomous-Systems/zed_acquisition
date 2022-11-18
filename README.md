Simplified alternative to https://github.com/stereolabs/zed-ros2-wrapper with a focus on image acquisition.

If raw image publishing is enabled via `general.publish_raw`, the raw calibration parameters are published to `(left|right)/camera_info_raw`.
This is done to avoid incompatibilities with common ROS image tools, which often assume `camera_info` to be in the same namespace as an image (i.e. `image_rect`).
If you want to use the raw camera info in your image processing node, remap `camera_info_raw:=camera_info` and `camera_info:=camera_info_rect` in your instance of `zed_acquisition::ZedCamera`.