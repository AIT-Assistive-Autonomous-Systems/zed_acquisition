#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <memory>
#include <utility>

namespace zed_acquisition
{

using sensor_msgs::msg::Image;
using std::placeholders::_1;
namespace enc = sensor_msgs::image_encodings;

class ImageEncode : public rclcpp::Node
{
private:
  rclcpp::Subscription<Image>::SharedPtr sub_in_;
  rclcpp::Publisher<Image>::SharedPtr pub_out_;

public:
  explicit ImageEncode(const rclcpp::NodeOptions & = rclcpp::NodeOptions());

  void image_callback(const Image::ConstSharedPtr);
};

ImageEncode::ImageEncode(const rclcpp::NodeOptions & options)
: rclcpp::Node("image_encode", options)
{
  pub_out_ = create_publisher<Image>("~/out", rclcpp::SensorDataQoS());
  sub_in_ = create_subscription<Image>(
    "~/in", rclcpp::SensorDataQoS(),
    std::bind(&ImageEncode::image_callback, this, _1));
  declare_parameter("encoding", enc::BGR8);
}

void ImageEncode::image_callback(const Image::ConstSharedPtr img)
{
  auto cv_img = cv_bridge::toCvShare(img, get_parameter("encoding").as_string());
  auto encoded_msg = cv_img->toImageMsg();
  pub_out_->publish(*encoded_msg);
}

} // namespace zed_acquisition

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(zed_acquisition::ImageEncode)
