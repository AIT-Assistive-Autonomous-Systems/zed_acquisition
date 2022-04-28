
// publish static IMU transform
  // Camera/IMU transform
  if (mCamRealModel != sl::MODEL::ZED) {
    mSlCamImuTransf = camInfo.sensors_configuration.camera_imu_transform;

    RCLCPP_DEBUG(
      get_logger(),
      "Camera-IMU Transform: \n %s",
      mSlCamImuTransf.getInfos().c_str());
  }


void ZedCamera::publishStaticImuFrameAndTopic()
{
  sl::Orientation sl_rot = mSlCamImuTransf.getOrientation();
  sl::Translation sl_tr = mSlCamImuTransf.getTranslation();

  mCameraImuTransfMgs = std::make_shared<geometry_msgs::msg::TransformStamped>();

  mCameraImuTransfMgs->header.stamp = get_clock()->now();

  mCameraImuTransfMgs->header.frame_id = mLeftCamFrameId;
  mCameraImuTransfMgs->child_frame_id = mImuFrameId;

  mCameraImuTransfMgs->transform.rotation.x = sl_rot.ox;
  mCameraImuTransfMgs->transform.rotation.y = sl_rot.oy;
  mCameraImuTransfMgs->transform.rotation.z = sl_rot.oz;
  mCameraImuTransfMgs->transform.rotation.w = sl_rot.ow;

  mCameraImuTransfMgs->transform.translation.x = sl_tr.x;
  mCameraImuTransfMgs->transform.translation.y = sl_tr.y;
  mCameraImuTransfMgs->transform.translation.z = sl_tr.z;

  // if (!mStaticImuTopicPublished) {
  //     rclcpp::QoS transf_qos = mSensQos;
  //     transf_qos.durability(
  //         RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL); // Latched topic
  //     transf_qos.keep_last(1);
  //     std::string cam_imu_tr_topic = mTopicRoot + "left_cam_imu_transform";
  //     mPubCamImuTransf = create_publisher<geometry_msgs::msg::TransformStamped>(
  //         cam_imu_tr_topic, transf_qos);

  //     mPubCamImuTransf->publish(*(mCameraImuTransfMgs.get()));

  //     RCLCPP_INFO_STREAM(get_logger(),
  //         "Advertised on topic: "
  //             << mPubCamImuTransf->get_topic_name() << " [LATCHED]");
  //     RCLCPP_INFO(get_logger(),
  //         "Camera-IMU Translation: \n %g %g %g",
  //         sl_tr.x,
  //         sl_tr.y,
  //         sl_tr.z);
  //     RCLCPP_INFO(get_logger(),
  //         "Camera-IMU Rotation: \n %s",
  //         sl_rot.getRotationMatrix().getInfos().c_str());

  //     mStaticImuTopicPublished = true;
  // }

  // Publish IMU TF as static TF
  if (!mPublishImuTF) {
    return;
  }

  if (mStaticImuFramePublished) {
    return;
  }

  // Publish transformation
  mStaticTfBroadcaster->sendTransform(*(mCameraImuTransfMgs.get()));

  RCLCPP_INFO_STREAM(
    get_logger(),
    "Published static TF: '" << mImuFrameId << "' -> '" <<
      mLeftCamFrameId << "'");

  mStaticImuFramePublished = true;
}



  bool ZedCamera::getSens2CameraTransform()
  {
    RCLCPP_DEBUG(
      get_logger(),
      "Getting static TF from '%s' to '%s'",
      mDepthFrameId.c_str(),
      mCameraFrameId.c_str());

    mSensor2CameraTransfValid = false;

    static bool first_error = true;

    // ----> Static transforms
    // Sensor to Camera Center
    try {
      // Save the transformation
      geometry_msgs::msg::TransformStamped s2c = mTfBuffer->lookupTransform(
        mDepthFrameId, mCameraFrameId, TIMEZERO_SYS, rclcpp::Duration(0.1));

      // Get the TF2 transformation
      // tf2::fromMsg(s2c.transform, mSensor2CameraTransf);
      geometry_msgs::msg::Transform in = s2c.transform;
      mSensor2CameraTransf.setOrigin(
        tf2::Vector3(in.translation.x, in.translation.y, in.translation.z));
      // w at the end in the constructor
      mSensor2CameraTransf.setRotation(
        tf2::Quaternion(
          in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w));

      double roll, pitch, yaw;
      tf2::Matrix3x3(mSensor2CameraTransf.getRotation()).getRPY(roll, pitch, yaw);

      RCLCPP_INFO(
        get_logger(),
        "Static transform Sensor to Camera Center [%s -> %s]",
        mDepthFrameId.c_str(),
        mCameraFrameId.c_str());
      RCLCPP_INFO(
        get_logger(),
        " * Translation: {%.3f,%.3f,%.3f}",
        mSensor2CameraTransf.getOrigin().x(),
        mSensor2CameraTransf.getOrigin().y(),
        mSensor2CameraTransf.getOrigin().z());
      RCLCPP_INFO(
        get_logger(),
        " * Rotation: {%.3f,%.3f,%.3f}",
        roll * RAD2DEG,
        pitch * RAD2DEG,
        yaw * RAD2DEG);
    } catch (tf2::TransformException & ex) {
      if (!first_error) {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), steady_clock, 1.0, "Transform error: %s", ex.what());
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          steady_clock,
          1.0,
          "The tf from '%s' to '%s' is not available.",
          mDepthFrameId.c_str(),
          mCameraFrameId.c_str());
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          steady_clock,
          1.0,
          "Note: one of the possible cause of the problem is the absense of an "
          "instance "
          "of the `robot_state_publisher` node publishing the correct static "
          "TF transformations "
          "or a modified URDF not correctly reproducing the ZED "
          "TF chain '%s' -> '%s' -> '%s'",
          mBaseFrameId.c_str(),
          mCameraFrameId.c_str(),
          mDepthFrameId.c_str());
        first_error = false;
      }

      mSensor2CameraTransf.setIdentity();
      return false;
    }

    // <---- Static transforms

    mSensor2CameraTransfValid = true;
    return true;
  }
