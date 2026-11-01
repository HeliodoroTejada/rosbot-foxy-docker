// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>

#include <depth_image_proc/conversions.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace astra_wrapper
{
class DepthToCloud : public rclcpp::Node
{
public:
  DepthToCloud()
  : Node("astra_depth_to_cloud")
  {
    const int queue_size = declare_parameter<int>("queue_size", 5);
    if (queue_size <= 0) {
      throw std::invalid_argument("queue_size must be positive");
    }
    const auto qos = rclcpp::SensorDataQoS();
    cloud_pub_ = create_publisher<Cloud>("points", qos);
    // Foxy's stock PointCloudXyzNode requests reliable image inputs. The
    // legacy Astra driver uses sensor-data QoS, so subscribe explicitly here
    // and reuse the standard depth_image_proc projection below.
    depth_sub_.subscribe(this, "image_rect", qos.get_rmw_qos_profile());
    info_sub_.subscribe(this, "camera_info", qos.get_rmw_qos_profile());
    sync_ = std::make_unique<message_filters::TimeSynchronizer<Image, Info>>(
      depth_sub_, info_sub_, queue_size);
    sync_->registerCallback(std::bind(
      &DepthToCloud::convert, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Publishing XYZ PointCloud2 on %s with sensor-data QoS",
      cloud_pub_->get_topic_name());
  }

private:
  using Image = sensor_msgs::msg::Image;
  using Info = sensor_msgs::msg::CameraInfo;
  using Cloud = sensor_msgs::msg::PointCloud2;

  void warn(const char * message)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", message);
  }

  void convert(const Image::ConstSharedPtr & depth, const Info::ConstSharedPtr & info)
  {
    if (cloud_pub_->get_subscription_count() == 0) {
      return;
    }
    if (depth->header.frame_id.empty() || depth->header.frame_id != info->header.frame_id) {
      warn("Depth and CameraInfo must have the same nonempty optical frame");
      return;
    }
    if (depth->width == 0 || depth->height == 0 ||
      depth->width != info->width || depth->height != info->height)
    {
      warn("Depth dimensions must be nonzero and match CameraInfo");
      return;
    }
    const bool metric = depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1;
    const bool millimetres = depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1;
    if (!metric && !millimetres) {
      warn("Depth encoding must be 32FC1 (metres) or 16UC1 (millimetres)");
      return;
    }
    const std::size_t bytes = metric ? sizeof(float) : sizeof(uint16_t);
    const uint16_t endian_probe = 1;
    const bool host_big_endian = reinterpret_cast<const uint8_t *>(&endian_probe)[0] == 0;
    if (static_cast<bool>(depth->is_bigendian) != host_big_endian ||
      depth->step < static_cast<std::size_t>(depth->width) * bytes ||
      depth->step % bytes != 0 ||
      depth->data.size() < static_cast<std::size_t>(depth->step) * depth->height)
    {
      warn("Depth data has unsupported byte order or an invalid row stride/data size");
      return;
    }
    if (!std::isfinite(info->p[0]) || !std::isfinite(info->p[5]) ||
      info->p[0] <= 0.0 || info->p[5] <= 0.0 ||
      !std::isfinite(info->p[2]) || !std::isfinite(info->p[6]))
    {
      warn("CameraInfo must contain valid calibrated projection intrinsics");
      return;
    }
    model_.fromCameraInfo(info);
    auto cloud = std::make_shared<Cloud>();
    cloud->header = depth->header;
    cloud->height = depth->height;
    cloud->width = depth->width;
    cloud->is_bigendian = host_big_endian;
    cloud->is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    if (metric) {
      depth_image_proc::convertDepth<float>(depth, cloud, model_);
    } else {
      depth_image_proc::convertDepth<uint16_t>(depth, cloud, model_);
    }
    cloud_pub_->publish(*cloud);
  }

  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
  message_filters::Subscriber<Image> depth_sub_;
  message_filters::Subscriber<Info> info_sub_;
  std::unique_ptr<message_filters::TimeSynchronizer<Image, Info>> sync_;
  image_geometry::PinholeCameraModel model_;
};
}  // namespace astra_wrapper

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<astra_wrapper::DepthToCloud>());
  rclcpp::shutdown();
  return 0;
}
