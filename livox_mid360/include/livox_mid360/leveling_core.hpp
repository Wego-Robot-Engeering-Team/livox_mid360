#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

namespace livox_mid360 {

struct FilterConfig {
  double gravity = 9.80665;
  double acceleration_tolerance = 1.5;
  double correction_time_constant = 0.5;
  double max_acceleration_innovation = 0.17;  // Radians from gyro-predicted gravity.
  double acceleration_change_threshold = 0.8;  // m/s^2 after removing gyro rotation.
  double stationary_recovery_delay = 1.0;  // Seconds of stable, low-gyro acceleration.
  double stationary_recovery_time_constant = 2.0;
  double max_imu_gap = 0.1;
  double initialization_max_gyro = 0.15;
  std::size_t initialization_samples = 50;
};

// Track the upward specific-force direction in the sensor frame. Heading is
// deliberately unobserved; the shortest rotation to +Z performs tilt leveling.
class GravityFilter {
public:
  explicit GravityFilter(FilterConfig config);
  std::optional<Eigen::Quaterniond> update(
    std::int64_t stamp_ns, const Eigen::Vector3d & acceleration,
    const Eigen::Vector3d & angular_velocity);
  void reset();
  bool ready() const {return ready_;}
  const Eigen::Quaterniond & gyro_rotation() const {return gyro_rotation_;}

private:
  FilterConfig config_;
  bool ready_ = false;
  std::optional<std::int64_t> last_stamp_;
  std::size_t sample_count_ = 0;
  Eigen::Vector3d acceleration_sum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_sum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d up_ = Eigen::Vector3d::UnitZ();
  std::optional<Eigen::Vector3d> previous_acceleration_;
  double acceleration_change_ema_ = 0.0;
  double stationary_duration_ = 0.0;
  Eigen::Quaterniond gyro_rotation_ = Eigen::Quaterniond::Identity();
};

struct AttitudeSample {
  std::int64_t stamp_ns;
  Eigen::Quaterniond rotation;
  Eigen::Quaterniond gyro_rotation = Eigen::Quaterniond::Identity();
};

struct CloudTimeRange {
  std::int64_t start_ns;
  std::int64_t end_ns;
};

std::optional<Eigen::Quaterniond> interpolate_attitude(
  const std::deque<AttitudeSample> & history, std::int64_t stamp_ns,
  double max_sample_distance);

// Rotate only x/y/z; keep intensity, vendor fields, layout, padding and NaNs.
bool rotate_cloud(
  const sensor_msgs::msg::PointCloud2 & input, const Eigen::Quaterniond & rotation,
  const std::string & output_frame, sensor_msgs::msg::PointCloud2 & output,
  std::string & error);

// Pinned Livox PointCloud2 timestamps are absolute nanoseconds, including at
// packet boundaries where host-arrival jitter can make point times unsorted.
bool cloud_time_range(
  const sensor_msgs::msg::PointCloud2 & input, CloudTimeRange & times, std::string & error);

// Correct rotation into the last point's sensor pose, then level that pose.
// Output header uses the maximum point time; original point timestamps remain.
// Translation is not estimated or corrected.
bool deskew_cloud(
  const sensor_msgs::msg::PointCloud2 & input, const std::deque<AttitudeSample> & history,
  double max_sample_distance, const std::string & output_frame,
  sensor_msgs::msg::PointCloud2 & output, std::string & error);

}  // namespace livox_mid360
