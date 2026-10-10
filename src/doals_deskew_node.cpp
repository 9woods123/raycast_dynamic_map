#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace raycast_static_map {
namespace {

template <typename T>
T param(ros::NodeHandle& nh, const std::string& name, const T& default_value) {
  T value;
  nh.param<T>(name, value, default_value);
  return value;
}

Eigen::Isometry3d odomPose(const nav_msgs::Odometry& odom) {
  const auto& position = odom.pose.pose.position;
  const auto& orientation = odom.pose.pose.orientation;
  Eigen::Quaterniond quaternion(orientation.w, orientation.x, orientation.y,
                               orientation.z);
  if (quaternion.norm() < 1e-6) quaternion = Eigen::Quaterniond::Identity();
  quaternion.normalize();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = quaternion.toRotationMatrix();
  pose.translation() =
      Eigen::Vector3d(position.x, position.y, position.z);
  return pose;
}

Eigen::Isometry3d interpolatePose(const Eigen::Isometry3d& start,
                                  const Eigen::Isometry3d& end,
                                  double alpha) {
  alpha = std::max(0.0, std::min(1.0, alpha));
  Eigen::Quaterniond start_q(start.linear());
  Eigen::Quaterniond end_q(end.linear());
  if (start_q.dot(end_q) < 0.0) end_q.coeffs() *= -1.0;

  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() =
      start_q.slerp(alpha, end_q).normalized().toRotationMatrix();
  result.translation() =
      (1.0 - alpha) * start.translation() + alpha * end.translation();
  return result;
}

const sensor_msgs::PointField* findField(const sensor_msgs::PointCloud2& cloud,
                                         const std::string& requested) {
  if (requested != "auto") {
    for (const auto& field : cloud.fields) {
      if (field.name == requested) return &field;
    }
    return nullptr;
  }
  const std::vector<std::string> candidates{"time", "t", "offset_time"};
  for (const auto& candidate : candidates) {
    for (const auto& field : cloud.fields) {
      if (field.name == candidate) return &field;
    }
  }
  return nullptr;
}

template <typename T>
void readField(const sensor_msgs::PointCloud2& cloud, const std::string& name,
               double scale, std::vector<double>* values) {
  sensor_msgs::PointCloud2ConstIterator<T> iterator(cloud, name);
  values->reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
  for (; iterator != iterator.end(); ++iterator) {
    values->push_back(static_cast<double>(*iterator) * scale);
  }
}

bool readTimeOffsets(const sensor_msgs::PointCloud2& cloud,
                     const std::string& requested_field,
                     double configured_scale, std::vector<double>* offsets,
                     std::string* resolved_field, double* resolved_scale) {
  const sensor_msgs::PointField* field = findField(cloud, requested_field);
  if (field == nullptr) return false;

  double scale = configured_scale;
  if (scale <= 0.0) {
    scale = (field->datatype == sensor_msgs::PointField::FLOAT32 ||
             field->datatype == sensor_msgs::PointField::FLOAT64)
                ? 1.0
                : 1e-9;
  }
  switch (field->datatype) {
    case sensor_msgs::PointField::FLOAT32:
      readField<float>(cloud, field->name, scale, offsets);
      break;
    case sensor_msgs::PointField::FLOAT64:
      readField<double>(cloud, field->name, scale, offsets);
      break;
    case sensor_msgs::PointField::UINT32:
      readField<std::uint32_t>(cloud, field->name, scale, offsets);
      break;
    case sensor_msgs::PointField::INT32:
      readField<std::int32_t>(cloud, field->name, scale, offsets);
      break;
    default:
      return false;
  }
  *resolved_field = field->name;
  *resolved_scale = scale;
  return true;
}

bool hasFloat32Xyz(const sensor_msgs::PointCloud2& cloud) {
  bool x = false, y = false, z = false;
  for (const auto& field : cloud.fields) {
    if (field.name == "x") x = field.datatype == sensor_msgs::PointField::FLOAT32;
    if (field.name == "y") y = field.datatype == sensor_msgs::PointField::FLOAT32;
    if (field.name == "z") z = field.datatype == sensor_msgs::PointField::FLOAT32;
  }
  return x && y && z;
}

}  // namespace

class DoalsDeskewNode {
 public:
  DoalsDeskewNode()
      : nh_(), private_nh_("~"),
        cloud_sub_(nh_, param<std::string>(private_nh_, "input_cloud_topic",
                                           "/doals/lidar_points"),
                   param<int>(private_nh_, "subscriber_queue_size", 20)),
        odom_sub_(nh_, param<std::string>(private_nh_, "input_odom_topic",
                                          "/doals/odom"),
                  param<int>(private_nh_, "subscriber_queue_size", 20)) {
    enabled_ = param<bool>(private_nh_, "enabled", true);
    point_time_field_ =
        param<std::string>(private_nh_, "point_time_field", "auto");
    point_time_scale_ = param<double>(private_nh_, "point_time_scale", 0.0);
    pose_bins_ = std::max(2, param<int>(private_nh_, "pose_bins", 128));
    max_scan_duration_ =
        param<double>(private_nh_, "max_scan_duration", 0.12);
    output_frame_id_ =
        param<std::string>(private_nh_, "output_frame_id", "");

    std::vector<double> translation{0.0, 0.0, 0.0};
    std::vector<double> rpy{0.0, 0.0, 0.0};
    private_nh_.param("sensor_extrinsic_translation", translation, translation);
    private_nh_.param("sensor_extrinsic_rpy", rpy, rpy);
    if (translation.size() != 3 || rpy.size() != 3) {
      throw std::runtime_error(
          "sensor extrinsic parameters must each contain 3 values");
    }
    body_T_lidar_ = Eigen::Isometry3d::Identity();
    body_T_lidar_.translation() =
        Eigen::Vector3d(translation[0], translation[1], translation[2]);
    body_T_lidar_.linear() =
        (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
            .toRotationMatrix();

    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        param<std::string>(private_nh_, "output_cloud_topic",
                           "/doals/lidar_points_deskewed"),
        2);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(
        param<std::string>(private_nh_, "output_odom_topic",
                           "/doals/odom_deskewed"),
        2);

    sync_.reset(new message_filters::Synchronizer<Policy>(
        Policy(param<int>(private_nh_, "sync_queue_size", 50)), cloud_sub_,
        odom_sub_));
    sync_->setMaxIntervalDuration(
        ros::Duration(param<double>(private_nh_, "sync_tolerance", 0.01)));
    sync_->registerCallback(
        boost::bind(&DoalsDeskewNode::callback, this, _1, _2));

    ROS_INFO_STREAM("DOALS deskew node ready; enabled=" << std::boolalpha
                    << enabled_ << ", pose bins=" << pose_bins_
                    << ", time field=" << point_time_field_);
  }

 private:
  using Policy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, nav_msgs::Odometry>;

  struct Frame {
    sensor_msgs::PointCloud2ConstPtr cloud;
    nav_msgs::OdometryConstPtr odom;
  };

  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                const nav_msgs::OdometryConstPtr& odom) {
    if (!enabled_) {
      publish(*cloud, *odom);
      return;
    }
    if (!pending_) {
      pending_.reset(new Frame{cloud, odom});
      return;
    }
    if (odom->header.stamp <= pending_->odom->header.stamp) {
      ROS_WARN("DOALS timestamp moved backwards; dropping buffered deskew frame");
      pending_.reset(new Frame{cloud, odom});
      return;
    }
    deskewAndPublish(*pending_->cloud, *pending_->odom, *odom);
    pending_.reset(new Frame{cloud, odom});
  }

  void publish(const sensor_msgs::PointCloud2& cloud,
               const nav_msgs::Odometry& odom) {
    sensor_msgs::PointCloud2 output_cloud = cloud;
    nav_msgs::Odometry output_odom = odom;
    output_odom.header.stamp = output_cloud.header.stamp;
    if (!output_frame_id_.empty()) output_cloud.header.frame_id = output_frame_id_;
    cloud_pub_.publish(output_cloud);
    odom_pub_.publish(output_odom);
  }

  void deskewAndPublish(const sensor_msgs::PointCloud2& cloud,
                        const nav_msgs::Odometry& reference_odom,
                        const nav_msgs::Odometry& next_odom) {
    if (!hasFloat32Xyz(cloud)) {
      ROS_ERROR_THROTTLE(2.0,
                         "DOALS deskew requires FLOAT32 x/y/z fields");
      return;
    }

    std::vector<double> offsets;
    std::string resolved_field;
    double resolved_scale = 0.0;
    if (!readTimeOffsets(cloud, point_time_field_, point_time_scale_, &offsets,
                         &resolved_field, &resolved_scale) ||
        offsets.size() != static_cast<std::size_t>(cloud.width) * cloud.height) {
      ROS_ERROR_THROTTLE(
          2.0, "DOALS deskew time field is missing, unsupported, or malformed");
      return;
    }

    const double pose_interval =
        (next_odom.header.stamp - reference_odom.header.stamp).toSec();
    if (pose_interval <= 1e-6) return;

    const Eigen::Isometry3d world_T_lidar_reference =
        odomPose(reference_odom) * body_T_lidar_;
    const Eigen::Isometry3d world_T_lidar_next =
        odomPose(next_odom) * body_T_lidar_;
    const Eigen::Isometry3d lidar_reference_T_world =
        world_T_lidar_reference.inverse();

    // Precompute relative transforms. Every point still uses its own timestamp,
    // but nearby timestamps share one interpolated pose to avoid 130k SLERPs.
    std::vector<Eigen::Isometry3d> reference_T_lidar_bins(pose_bins_);
    for (int bin = 0; bin < pose_bins_; ++bin) {
      const double alpha = static_cast<double>(bin) / (pose_bins_ - 1);
      reference_T_lidar_bins[bin] =
          lidar_reference_T_world *
          interpolatePose(world_T_lidar_reference, world_T_lidar_next, alpha);
    }

    sensor_msgs::PointCloud2 output = cloud;
    if (!output_frame_id_.empty()) output.header.frame_id = output_frame_id_;
    sensor_msgs::PointCloud2Iterator<float> x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> z(output, "z");

    const double cloud_to_pose_offset =
        (cloud.header.stamp - reference_odom.header.stamp).toSec();
    double maximum_offset = 0.0;
    std::size_t clamped_points = 0;
    for (std::size_t index = 0; index < offsets.size();
         ++index, ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        continue;
      }
      maximum_offset = std::max(maximum_offset, offsets[index]);
      double alpha = (cloud_to_pose_offset + offsets[index]) / pose_interval;
      if (alpha < 0.0 || alpha > 1.0) {
        ++clamped_points;
        alpha = std::max(0.0, std::min(1.0, alpha));
      }
      const int bin = std::max(
          0, std::min(pose_bins_ - 1,
                      static_cast<int>(std::lround(alpha * (pose_bins_ - 1)))));
      const Eigen::Vector3d corrected =
          reference_T_lidar_bins[bin] * Eigen::Vector3d(*x, *y, *z);
      *x = static_cast<float>(corrected.x());
      *y = static_cast<float>(corrected.y());
      *z = static_cast<float>(corrected.z());
    }

    if (maximum_offset > max_scan_duration_) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "point time reaches " << maximum_offset
                                      << " s, above max_scan_duration="
                                      << max_scan_duration_ << " s");
    }
    if (clamped_points > 0) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "clamped " << clamped_points
                           << " point timestamps outside adjacent poses");
    }

    nav_msgs::Odometry output_odom = reference_odom;
    output_odom.header.stamp = output.header.stamp;
    cloud_pub_.publish(output);
    odom_pub_.publish(output_odom);
    ++published_frames_;
    ROS_INFO_STREAM_THROTTLE(
        2.0, "deskewed frames=" << published_frames_ << ", points="
                                 << offsets.size() << ", field="
                                 << resolved_field << ", scale="
                                 << resolved_scale << ", scan="
                                 << maximum_offset << " s");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;
  ros::Publisher cloud_pub_;
  ros::Publisher odom_pub_;
  std::unique_ptr<Frame> pending_;
  Eigen::Isometry3d body_T_lidar_ = Eigen::Isometry3d::Identity();
  std::string point_time_field_ = "auto";
  std::string output_frame_id_;
  bool enabled_ = true;
  int pose_bins_ = 128;
  double point_time_scale_ = 0.0;
  double max_scan_duration_ = 0.12;
  std::size_t published_frames_ = 0;
};

}  // namespace raycast_static_map

int main(int argc, char** argv) {
  ros::init(argc, argv, "doals_deskew");
  try {
    raycast_static_map::DoalsDeskewNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Failed to start DOALS deskew node: " << error.what());
    return 1;
  }
  return 0;
}
