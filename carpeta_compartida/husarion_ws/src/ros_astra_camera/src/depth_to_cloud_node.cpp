// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
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
    pixel_stride_ = declare_parameter<int>("pixel_stride", 2);
    max_rate_ = declare_parameter<double>("max_rate", 5.0);
    if (pixel_stride_ <= 0 || !std::isfinite(max_rate_) || max_rate_ < 0.0) {
      throw std::invalid_argument("pixel_stride must be positive and max_rate finite and >= 0");
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
    RCLCPP_INFO(get_logger(),
      "Publishing XYZ PointCloud2 on %s: pixel_stride=%d, max_rate=%.2f Hz (0=unlimited)",
      cloud_pub_->get_topic_name(), pixel_stride_, max_rate_);
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
    // Drop excess frames before copying depth pixels or allocating/projecting a cloud.
    // A steady clock keeps this limit independent of sensor/ROS clock corrections.
    const auto now = std::chrono::steady_clock::now();
    if (have_published_ && max_rate_ > 0.0 &&
      std::chrono::duration<double>(now - last_publication_).count() < 1.0 / max_rate_)
    {
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
    Image::ConstSharedPtr sampled_depth = depth;
    const image_geometry::PinholeCameraModel * projection = &model_;
    if (pixel_stride_ > 1) {
      // Sample (0, 0), (stride, 0), ... directly; never build a full-resolution cloud.
      // Ceiling division also retains the final sampled row/column of odd-sized images.
      auto sampled = std::make_shared<Image>();
      sampled->header = depth->header;
      sampled->encoding = depth->encoding;
      sampled->is_bigendian = depth->is_bigendian;
      sampled->width = 1 + (depth->width - 1) / pixel_stride_;
      sampled->height = 1 + (depth->height - 1) / pixel_stride_;
      sampled->step = sampled->width * bytes;
      sampled->data.resize(static_cast<std::size_t>(sampled->step) * sampled->height);
      for (uint32_t v = 0; v < sampled->height; ++v) {
        for (uint32_t u = 0; u < sampled->width; ++u) {
          const auto source = static_cast<std::size_t>(v) * pixel_stride_ * depth->step +
            static_cast<std::size_t>(u) * pixel_stride_ * bytes;
          const auto target = static_cast<std::size_t>(v) * sampled->step + u * bytes;
          std::memcpy(sampled->data.data() + target, depth->data.data() + source, bytes);
        }
      }
      // Scale the effective rectified intrinsics, including any input ROI/binning.
      // Sampling without averaging has no half-pixel shift: u_input = stride*u_output.
      Info sampled_info;
      sampled_info.header = info->header;
      sampled_info.width = sampled->width;
      sampled_info.height = sampled->height;
      const double fx = model_.fx() / pixel_stride_, fy = model_.fy() / pixel_stride_;
      const double cx = model_.cx() / pixel_stride_, cy = model_.cy() / pixel_stride_;
      sampled_info.k = {{fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0}};
      sampled_info.r = {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
      sampled_info.p = {{fx, 0.0, cx, model_.Tx() / pixel_stride_,
        0.0, fy, cy, model_.Ty() / pixel_stride_, 0.0, 0.0, 1.0, 0.0}};
      sampled_model_.fromCameraInfo(sampled_info);
      sampled_depth = sampled;
      projection = &sampled_model_;
    }
    auto cloud = std::make_shared<Cloud>();
    cloud->header = depth->header;
    cloud->height = sampled_depth->height;
    cloud->width = sampled_depth->width;
    cloud->is_bigendian = host_big_endian;
    cloud->is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    if (metric) {
      depth_image_proc::convertDepth<float>(sampled_depth, cloud, *projection);
    } else {
      depth_image_proc::convertDepth<uint16_t>(sampled_depth, cloud, *projection);
    }
    cloud_pub_->publish(*cloud);
    last_publication_ = now;
    have_published_ = true;
  }

  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
  message_filters::Subscriber<Image> depth_sub_;
  message_filters::Subscriber<Info> info_sub_;
  std::unique_ptr<message_filters::TimeSynchronizer<Image, Info>> sync_;
  image_geometry::PinholeCameraModel model_;
  image_geometry::PinholeCameraModel sampled_model_;
  int pixel_stride_;
  double max_rate_;
  bool have_published_ = false;
  std::chrono::steady_clock::time_point last_publication_;
};
}  // namespace astra_wrapper

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<astra_wrapper::DepthToCloud>());
  rclcpp::shutdown();
  return 0;
}
