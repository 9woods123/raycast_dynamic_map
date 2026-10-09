#include "raycast_static_map/raycast_mapper.hpp"
#include "raycast_static_map/object_tracker.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
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
  const auto& p = odom.pose.pose.position;
  const auto& q = odom.pose.pose.orientation;
  Eigen::Quaterniond quaternion(q.w, q.x, q.y, q.z);
  if (quaternion.norm() < 1e-6) quaternion = Eigen::Quaterniond::Identity();
  quaternion.normalize();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = quaternion.toRotationMatrix();
  pose.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  return pose;
}

sensor_msgs::PointCloud2 xyzCloud(const std::vector<Eigen::Vector3d>& points,
                                  const std_msgs::Header& header) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.points.resize(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    cloud.points[i].x = static_cast<float>(points[i].x());
    cloud.points[i].y = static_cast<float>(points[i].y());
    cloud.points[i].z = static_cast<float>(points[i].z());
  }
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header = header;
  return message;
}

sensor_msgs::PointCloud2 mapCloud(const std::vector<MapPoint>& points,
                                  const std_msgs::Header& header) {
  sensor_msgs::PointCloud2 message;
  message.header = header;
  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2Fields(6, "x", 1, sensor_msgs::PointField::FLOAT32,
                               "y", 1, sensor_msgs::PointField::FLOAT32,
                               "z", 1, sensor_msgs::PointField::FLOAT32,
                               "occupancy", 1, sensor_msgs::PointField::FLOAT32,
                               "esdf", 1, sensor_msgs::PointField::FLOAT32,
                               "tsdf", 1, sensor_msgs::PointField::FLOAT32);
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(message, "x"), y(message, "y"),
      z(message, "z"), occupancy(message, "occupancy"), esdf(message, "esdf"),
      tsdf(message, "tsdf");
  for (const auto& point : points) {
    *x = static_cast<float>(point.position.x());
    *y = static_cast<float>(point.position.y());
    *z = static_cast<float>(point.position.z());
    *occupancy = static_cast<float>(point.probability);
    *esdf = static_cast<float>(point.esdf_distance);
    *tsdf = static_cast<float>(point.tsdf);
    ++x; ++y; ++z; ++occupancy; ++esdf; ++tsdf;
  }
  return message;
}

// Deterministic per-object color. Stepping the hue by the golden ratio gives
// well separated, stable colors for consecutive track ids.
std_msgs::ColorRGBA objectColor(int id, float alpha, float value = 1.0f) {
  const double hue =
      std::fmod(0.61803398875 * (static_cast<double>(id) + 0.5), 1.0);
  const double h = hue * 6.0;
  const int sector = static_cast<int>(std::floor(h)) % 6;
  const double f = h - std::floor(h);
  double r = 0.0, g = 0.0, b = 0.0;
  switch (sector) {
    case 0: r = 1.0; g = f; b = 0.0; break;
    case 1: r = 1.0 - f; g = 1.0; b = 0.0; break;
    case 2: r = 0.0; g = 1.0; b = f; break;
    case 3: r = 0.0; g = 1.0 - f; b = 1.0; break;
    case 4: r = f; g = 0.0; b = 1.0; break;
    default: r = 1.0; g = 0.0; b = 1.0 - f; break;
  }
  std_msgs::ColorRGBA color;
  color.r = static_cast<float>(r) * value;
  color.g = static_cast<float>(g) * value;
  color.b = static_cast<float>(b) * value;
  color.a = alpha;
  return color;
}

}  // namespace

class RaycastStaticMapNode {
 public:
  RaycastStaticMapNode()
      : nh_(), private_nh_("~"),
        cloud_sub_(nh_, param<std::string>(private_nh_, "cloud_topic", "/cloud_registered"),
                   param<int>(private_nh_, "subscriber_queue_size", 10)),
        odom_sub_(nh_, param<std::string>(private_nh_, "odom_topic", "/Odometry"),
                  param<int>(private_nh_, "subscriber_queue_size", 10)) {
    MapperConfig config;
    config.resolution = param<double>(private_nh_, "resolution", 0.20);
    config.min_range = param<double>(private_nh_, "min_range", 1.0);
    config.max_range = param<double>(private_nh_, "max_range", 80.0);
    config.prob_hit = param<double>(private_nh_, "prob_hit", 0.70);
    config.prob_miss = param<double>(private_nh_, "prob_miss", 0.40);
    config.prob_min = param<double>(private_nh_, "prob_min", 0.12);
    config.prob_max = param<double>(private_nh_, "prob_max", 0.97);
    config.occupied_probability = param<double>(private_nh_, "occupied_probability", 0.60);
    config.use_tsdf = param<bool>(private_nh_, "use_tsdf", false);
    config.tsdf_truncation = param<double>(private_nh_, "tsdf_truncation", 0.60);
    config.tsdf_surface_band = param<double>(private_nh_, "tsdf_surface_band", 0.10);
    config.tsdf_max_weight = param<double>(private_nh_, "tsdf_max_weight", 30.0);
    config.use_dynablox = param<bool>(private_nh_, "use_dynablox", true);
    config.dynamic_z_use_map_frame =
        param<bool>(private_nh_, "dynamic_z_use_map_frame", false);
    config.dynamic_z_min = param<double>(private_nh_, "dynamic_z_min", -2.0);
    config.dynamic_z_max = param<double>(private_nh_, "dynamic_z_max", 2.5);
    config.ever_free_burn_in_period = param<int>(private_nh_, "ever_free_burn_in_period", 5);
    config.ever_free_temporal_buffer_frames = param<int>(private_nh_, "ever_free_temporal_buffer_frames", 2);
    config.ever_free_reset_occupied_frames = param<int>(private_nh_, "ever_free_reset_occupied_frames", 150);
    config.ever_free_neighbor_connectivity = param<int>(private_nh_, "ever_free_neighbor_connectivity", 26);
    config.ever_free_max_free_probability =
        param<double>(private_nh_, "ever_free_max_free_probability", 0.0);
    config.cluster_connectivity = param<int>(private_nh_, "cluster_connectivity", 6);
    config.cluster_grow_clusters_twice = param<bool>(private_nh_, "cluster_grow_clusters_twice", false);
    config.cluster_min_separation = param<double>(private_nh_, "cluster_min_separation", 0.0);
    config.cluster_min_extent = param<double>(private_nh_, "cluster_min_extent", 0.0);
    config.cluster_max_extent = param<double>(private_nh_, "cluster_max_extent", 200000.0);
    config.cluster_min_points = param<int>(private_nh_, "cluster_min_points", 20);
    config.cluster_max_points = param<int>(private_nh_, "cluster_max_points", 200000);
    config.esdf_max_distance = param<double>(private_nh_, "esdf_max_distance", 5.0);
    config.esdf_update_every_n_frames = param<int>(private_nh_, "esdf_update_every_n_frames", 1);
    mapper_.reset(new RaycastMapper(config));

    TrackerConfig tracker_config;
    tracker_config.max_association_distance =
        param<double>(private_nh_, "track_max_association_distance", 1.0);
    tracker_config.min_track_duration =
        param<int>(private_nh_, "track_min_track_duration", 2);
    tracker_config.max_misses = param<int>(private_nh_, "track_max_misses", 5);
    tracker_config.kalman_process_noise =
        param<double>(private_nh_, "track_kalman_process_noise", 2.0);
    tracker_config.kalman_measurement_noise =
        param<double>(private_nh_, "track_kalman_measurement_noise", 0.5);
    tracker_config.velocity_horizon =
        param<double>(private_nh_, "track_velocity_horizon", 1.0);
    tracker_config.max_velocity = param<double>(private_nh_, "track_max_velocity", 20.0);
    tracker_config.min_velocity = param<double>(private_nh_, "track_min_velocity", 0.10);
    tracker_config.max_tracks = param<int>(private_nh_, "track_max_tracks", 500);
    tracker_.reset(new ObjectTracker(tracker_config));
    use_tracking_ = param<bool>(private_nh_, "use_tracking", true);
    track_publish_unconfirmed_ =
        param<bool>(private_nh_, "track_publish_unconfirmed", false);

    map_frame_ = param<std::string>(private_nh_, "map_frame", "camera_init");
    input_cloud_in_world_ = param<bool>(private_nh_, "input_cloud_in_world", true);
    publish_every_n_frames_ = std::max(1, param<int>(private_nh_, "publish_every_n_frames", 1));
    publish_known_voxels_ = param<bool>(private_nh_, "publish_known_voxels", true);
    publish_2d_grid_ = param<bool>(private_nh_, "publish_2d_grid", true);
    grid_z_min_ = param<double>(private_nh_, "grid_z_min", -1.0);
    grid_z_max_ = param<double>(private_nh_, "grid_z_max", 2.0);
    grid_max_cells_ = param<int>(private_nh_, "grid_max_cells", 4000000);

    std::vector<double> translation{0.0, 0.0, 0.0}, rpy{0.0, 0.0, 0.0};
    private_nh_.param("sensor_extrinsic_translation", translation, translation);
    private_nh_.param("sensor_extrinsic_rpy", rpy, rpy);
    if (translation.size() != 3 || rpy.size() != 3) {
      throw std::runtime_error("sensor extrinsic parameters must each contain 3 values");
    }
    body_T_sensor_ = Eigen::Isometry3d::Identity();
    body_T_sensor_.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);
    body_T_sensor_.linear() =
        (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();

    occupied_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("occupied_voxels", 1, true);
    known_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("known_voxels", 1, true);
    dynamic_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("dynamic_points", 1);
    static_scan_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("static_points", 1);
    grid_pub_ = private_nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1, true);
    objects_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("dynamic_objects", 1);
    reset_service_ = private_nh_.advertiseService("reset", &RaycastStaticMapNode::reset, this);

    sync_.reset(new message_filters::Synchronizer<Policy>(
        Policy(param<int>(private_nh_, "sync_queue_size", 30)), cloud_sub_, odom_sub_));
    sync_->setMaxIntervalDuration(ros::Duration(param<double>(private_nh_, "sync_tolerance", 0.05)));
    sync_->registerCallback(boost::bind(&RaycastStaticMapNode::callback, this, _1, _2));
    ROS_INFO_STREAM("raycast_static_map ready; occupancy criterion="
                    << (config.use_tsdf ? "TSDF" : "probability")
                    << ", Dynablox=" << (config.use_dynablox ? "on" : "off")
                    << ", tracking=" << (use_tracking_ ? "on" : "off")
                    << ", world-aligned input=" << std::boolalpha << input_cloud_in_world_);
  }

 private:
  using Policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                                 nav_msgs::Odometry>;

  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_message,
                const nav_msgs::OdometryConstPtr& odom_message) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_message, cloud);
    const Eigen::Isometry3d world_T_sensor = odomPose(*odom_message) * body_T_sensor_;
    const Eigen::Isometry3d sensor_T_world = world_T_sensor.inverse();
    std::vector<Eigen::Vector3d> world_points, sensor_points;
    world_points.reserve(cloud.size());
    sensor_points.reserve(cloud.size());
    for (const auto& point : cloud.points) {
      const Eigen::Vector3d input(point.x, point.y, point.z);
      if (!input.allFinite()) continue;
      if (input_cloud_in_world_) {
        world_points.push_back(input);
        sensor_points.push_back(sensor_T_world * input);
      } else {
        sensor_points.push_back(input);
        world_points.push_back(world_T_sensor * input);
      }
    }

    const FrameResult result = mapper_->integrate(world_points, sensor_points,
                                                   world_T_sensor.translation());
    std_msgs::Header header = cloud_message->header;
    header.frame_id = map_frame_;
    static_scan_pub_.publish(xyzCloud(result.static_points_world, header));
    dynamic_pub_.publish(xyzCloud(result.dynamic_points_world, header));
    if (use_tracking_) {
      const double stamp = cloud_message->header.stamp.toSec();
      const std::vector<TrackedObject> objects =
          tracker_->update(result.dynamic_clusters, stamp);
      publishObjects(objects, header);
    }
    if (mapper_->frameIndex() % publish_every_n_frames_ != 0) return;

    const auto occupied = mapper_->occupiedVoxels();
    occupied_pub_.publish(mapCloud(occupied, header));
    if (publish_known_voxels_) known_pub_.publish(mapCloud(mapper_->knownVoxels(), header));
    if (publish_2d_grid_) publishGrid(occupied, header);
    ROS_INFO_STREAM_THROTTLE(2.0, "hash voxels=" << mapper_->voxelCount()
                             << ", occupied=" << occupied.size()
                             << ", dynamic points=" << result.dynamic_points_world.size());
  }

  // Publish tracked dynamic objects as an RViz MarkerArray: one bounding box,
  // one text label (id + speed), one velocity arrow and one predicted-position
  // sphere per object. A leading DELETEALL clears stale markers each frame.
  void publishObjects(const std::vector<TrackedObject>& objects,
                      const std_msgs::Header& header) {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);

    const double resolution = mapper_->config().resolution;
    int index = 0;
    for (const auto& object : objects) {
      if (!track_publish_unconfirmed_ && !object.confirmed) continue;
      const Eigen::Vector3d size = object.max_corner - object.min_corner;
      const Eigen::Vector3d center =
          0.5 * (object.min_corner + object.max_corner);
      const double speed = object.speed;
      // Each track gets its own stable color so different objects are easy to
      // tell apart; the speed is reported in the text label instead.
      const std_msgs::ColorRGBA color = objectColor(object.id, 1.0f);
      const std_msgs::ColorRGBA dim_color = objectColor(object.id, 0.5f, 0.8f);

      // Wireframe bounding box (12 edges as a LINE_LIST) instead of a solid cube.
      visualization_msgs::Marker box;
      box.header = header;
      box.ns = "dynamic_object_box";
      box.id = index;
      box.type = visualization_msgs::Marker::LINE_LIST;
      box.action = visualization_msgs::Marker::ADD;
      box.pose.position.x = center.x();
      box.pose.position.y = center.y();
      box.pose.position.z = center.z();
      box.pose.orientation.w = 1.0;
      box.scale.x = 0.05;  // Line width in meters.
      box.color = color;
      box.lifetime = ros::Duration(0.5);
      const double hx = 0.5 * (size.x() + resolution);
      const double hy = 0.5 * (size.y() + resolution);
      const double hz = 0.5 * (size.z() + resolution);
      const Eigen::Vector3d corners[8] = {
          {-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, hy, -hz}, {-hx, hy, -hz},
          {-hx, -hy, hz},  {hx, -hy, hz},  {hx, hy, hz},  {-hx, hy, hz}};
      static const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0},
                                       {4, 5}, {5, 6}, {6, 7}, {7, 4},
                                       {0, 4}, {1, 5}, {2, 6}, {3, 7}};
      for (const auto& edge : edges) {
        geometry_msgs::Point start;
        start.x = corners[edge[0]].x();
        start.y = corners[edge[0]].y();
        start.z = corners[edge[0]].z();
        geometry_msgs::Point end;
        end.x = corners[edge[1]].x();
        end.y = corners[edge[1]].y();
        end.z = corners[edge[1]].z();
        box.points.push_back(start);
        box.points.push_back(end);
      }
      markers.markers.push_back(box);

      std::ostringstream label_text;
      label_text << "id " << object.id << std::fixed << std::setprecision(1)
                 << "  " << speed << " m/s";
      visualization_msgs::Marker label;
      label.header = header;
      label.ns = "dynamic_object_label";
      label.id = index;
      label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::Marker::ADD;
      label.pose.position.x = center.x();
      label.pose.position.y = center.y();
      label.pose.position.z = object.max_corner.z() + 0.4;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.4;
      label.color = color;
      label.text = label_text.str();
      label.lifetime = ros::Duration(0.5);
      markers.markers.push_back(label);

      if (speed > 1e-3) {
        visualization_msgs::Marker arrow;
        arrow.header = header;
        arrow.ns = "dynamic_object_velocity";
        arrow.id = index;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.pose.orientation.w = 1.0;
        arrow.scale.x = 0.06;
        arrow.scale.y = 0.12;
        arrow.scale.z = 0.18;
        arrow.color = color;
        arrow.lifetime = ros::Duration(0.5);
        geometry_msgs::Point start, end;
        start.x = object.centroid.x();
        start.y = object.centroid.y();
        start.z = object.centroid.z();
        end.x = object.centroid.x() + object.velocity.x();
        end.y = object.centroid.y() + object.velocity.y();
        end.z = object.centroid.z() + object.velocity.z();
        arrow.points.push_back(start);
        arrow.points.push_back(end);
        markers.markers.push_back(arrow);
      }

      visualization_msgs::Marker prediction;
      prediction.header = header;
      prediction.ns = "dynamic_object_prediction";
      prediction.id = index;
      prediction.type = visualization_msgs::Marker::SPHERE;
      prediction.action = visualization_msgs::Marker::ADD;
      prediction.pose.position.x = object.predicted_position.x();
      prediction.pose.position.y = object.predicted_position.y();
      prediction.pose.position.z = object.predicted_position.z();
      prediction.pose.orientation.w = 1.0;
      prediction.scale.x = 0.3;
      prediction.scale.y = 0.3;
      prediction.scale.z = 0.3;
      prediction.color = dim_color;
      prediction.lifetime = ros::Duration(0.5);
      markers.markers.push_back(prediction);

      ++index;
    }
    objects_pub_.publish(markers);
  }

  void publishGrid(const std::vector<MapPoint>& occupied,
                   const std_msgs::Header& header) {
    if (occupied.empty()) return;
    const double resolution = mapper_->config().resolution;
    int min_x = std::numeric_limits<int>::max(), min_y = min_x;
    int max_x = std::numeric_limits<int>::min(), max_y = max_x;
    for (const auto& point : occupied) {
      if (point.position.z() < grid_z_min_ || point.position.z() > grid_z_max_) continue;
      const int x = static_cast<int>(std::floor(point.position.x() / resolution));
      const int y = static_cast<int>(std::floor(point.position.y() / resolution));
      min_x = std::min(min_x, x); max_x = std::max(max_x, x);
      min_y = std::min(min_y, y); max_y = std::max(max_y, y);
    }
    if (min_x > max_x || min_y > max_y) return;
    const std::size_t width = static_cast<std::size_t>(max_x - min_x + 1);
    const std::size_t height = static_cast<std::size_t>(max_y - min_y + 1);
    if (width * height > static_cast<std::size_t>(std::max(1, grid_max_cells_))) {
      ROS_WARN_THROTTLE(5.0, "2D grid exceeds grid_max_cells; publication skipped");
      return;
    }
    nav_msgs::OccupancyGrid grid;
    grid.header = header;
    grid.info.resolution = resolution;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = min_x * resolution;
    grid.info.origin.position.y = min_y * resolution;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(width * height, -1);
    for (const auto& point : occupied) {
      if (point.position.z() < grid_z_min_ || point.position.z() > grid_z_max_) continue;
      const int x = static_cast<int>(std::floor(point.position.x() / resolution)) - min_x;
      const int y = static_cast<int>(std::floor(point.position.y() / resolution)) - min_y;
      const std::size_t index = static_cast<std::size_t>(y) * width + x;
      // In TSDF mode the occupied set is already decided by the signed distance,
      // so mark it fully occupied; otherwise keep the probability confidence.
      const int8_t cell_value = mapper_->config().use_tsdf
          ? 100
          : static_cast<int8_t>(std::round(100.0 * point.probability));
      grid.data[index] = std::max<int8_t>(grid.data[index], cell_value);
    }
    grid_pub_.publish(grid);
  }

  bool reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    mapper_->reset();
    if (tracker_) tracker_->reset();
    ROS_INFO("ray-cast occupancy and ESDF maps reset");
    return true;
  }

  ros::NodeHandle nh_, private_nh_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;
  std::unique_ptr<RaycastMapper> mapper_;
  std::unique_ptr<ObjectTracker> tracker_;
  ros::Publisher occupied_pub_, known_pub_, dynamic_pub_, static_scan_pub_, grid_pub_;
  ros::Publisher objects_pub_;
  ros::ServiceServer reset_service_;
  Eigen::Isometry3d body_T_sensor_ = Eigen::Isometry3d::Identity();
  std::string map_frame_;
  bool input_cloud_in_world_ = true, publish_known_voxels_ = true, publish_2d_grid_ = true;
  bool use_tracking_ = true, track_publish_unconfirmed_ = false;
  int publish_every_n_frames_ = 1, grid_max_cells_ = 4000000;
  double grid_z_min_ = -1.0, grid_z_max_ = 2.0;
};

}  // namespace raycast_static_map

int main(int argc, char** argv) {
  ros::init(argc, argv, "raycast_static_map");
  try {
    raycast_static_map::RaycastStaticMapNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL_STREAM("Failed to start raycast_static_map: " << exception.what());
    return 1;
  }
  return 0;
}
