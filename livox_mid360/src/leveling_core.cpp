#include "livox_mid360/leveling_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace livox_mid360 {

GravityFilter::GravityFilter(FilterConfig config) : config_(config) {
  if (!std::isfinite(config.gravity) || config.gravity <= 0 ||
    !std::isfinite(config.acceleration_tolerance) || config.acceleration_tolerance <= 0 ||
    !std::isfinite(config.correction_time_constant) || config.correction_time_constant <= 0 ||
    !std::isfinite(config.max_acceleration_innovation) ||
    config.max_acceleration_innovation <= 0 || config.max_acceleration_innovation >= M_PI ||
    !std::isfinite(config.acceleration_change_threshold) || config.acceleration_change_threshold <= 0 ||
    !std::isfinite(config.stationary_recovery_delay) || config.stationary_recovery_delay <= 0 ||
    !std::isfinite(config.stationary_recovery_time_constant) ||
    config.stationary_recovery_time_constant <= 0 ||
    !std::isfinite(config.max_imu_gap) || config.max_imu_gap <= 0 ||
    !std::isfinite(config.initialization_max_gyro) || config.initialization_max_gyro <= 0 ||
    config.initialization_samples == 0)
  {
    throw std::invalid_argument("Invalid gravity filter parameters");
  }
}

void GravityFilter::reset() {
  ready_ = false;
  last_stamp_.reset();
  sample_count_ = 0;
  acceleration_sum_.setZero();
  gyro_sum_.setZero();
  gyro_bias_.setZero();
  up_ = Eigen::Vector3d::UnitZ();
  previous_acceleration_.reset();
  acceleration_change_ema_ = 0.0;
  stationary_duration_ = 0.0;
  gyro_rotation_ = Eigen::Quaterniond::Identity();
}

std::optional<Eigen::Quaterniond> GravityFilter::update(
  std::int64_t stamp_ns, const Eigen::Vector3d & acceleration,
  const Eigen::Vector3d & angular_velocity)
{
  if (!acceleration.allFinite() || !angular_velocity.allFinite()) {
    return std::nullopt;
  }
  double dt = 0.0;
  if (last_stamp_) {
    dt = static_cast<double>(stamp_ns - *last_stamp_) * 1e-9;
    if (dt == 0.0) {return std::nullopt;}
    if (dt < 0.0 || dt > config_.max_imu_gap) {reset(); dt = 0.0;}
  }
  last_stamp_ = stamp_ns;
  const double norm = acceleration.norm();
  const bool valid_gravity = norm > 1e-6 &&
    std::abs(norm - config_.gravity) <= config_.acceleration_tolerance;
  if (!ready_) {
    if (!valid_gravity || angular_velocity.norm() > config_.initialization_max_gyro) {
      sample_count_ = 0;
      acceleration_sum_.setZero();
      gyro_sum_.setZero();
      return std::nullopt;
    }
    acceleration_sum_ += acceleration;
    gyro_sum_ += angular_velocity;
    if (++sample_count_ < config_.initialization_samples) {return std::nullopt;}
    if (acceleration_sum_.norm() < 1e-6) {reset(); return std::nullopt;}
    up_ = acceleration_sum_.normalized();
    gyro_bias_ = gyro_sum_ / static_cast<double>(sample_count_);
    gyro_rotation_ = Eigen::Quaterniond::Identity();
    stationary_duration_ = 0.0;
    ready_ = true;
  } else {
    const Eigen::Vector3d corrected_gyro = angular_velocity - gyro_bias_;
    const Eigen::Vector3d delta = corrected_gyro * dt;
    const double angle = delta.norm();
    Eigen::Quaterniond inverse_delta = Eigen::Quaterniond::Identity();
    if (angle > 1e-12) {
      // A fixed world direction expressed in a rotating sensor moves inversely.
      const Eigen::Quaterniond forward_delta(Eigen::AngleAxisd(angle, delta / angle));
      inverse_delta = forward_delta.conjugate();
      up_ = inverse_delta * up_;
      // Keep full gyro rotation separate from the heading-free gravity correction.
      gyro_rotation_ = (gyro_rotation_ * forward_delta).normalized();
    }
    // Comparing raw consecutive samples would confuse genuine rotation with
    // translational acceleration. First rotate the previous sample by the gyro.
    const double acceleration_change = previous_acceleration_ ?
      (acceleration - inverse_delta * *previous_acceleration_).norm() :
      std::numeric_limits<double>::infinity();
    // Judge stationary noise over 100 ms instead of requiring every single
    // noisy sample to pass a tighter threshold for the entire recovery dwell.
    if (std::isfinite(acceleration_change)) {
      const double smoothing = 1.0 - std::exp(-dt / 0.1);
      acceleration_change_ema_ += smoothing * (acceleration_change - acceleration_change_ema_);
    } else {
      acceleration_change_ema_ = config_.acceleration_change_threshold;
    }
    const bool stable = valid_gravity &&
      corrected_gyro.norm() <= 0.25 * config_.initialization_max_gyro &&
      acceleration_change <= config_.acceleration_change_threshold &&
      acceleration_change_ema_ <= 0.25 * config_.acceleration_change_threshold;
    stationary_duration_ = stable ? stationary_duration_ + dt : 0.0;
    if (valid_gravity) {
      const auto trust = [](double ratio) {
          return std::max(0.0, 1.0 - ratio * ratio);
        };
      const Eigen::Vector3d measured_up = acceleration / norm;
      const double innovation = std::acos(std::clamp(up_.dot(measured_up), -1.0, 1.0));
      const double norm_trust = trust(std::abs(norm - config_.gravity) /
        config_.acceleration_tolerance);
      double correction_rate = norm_trust *
        trust(innovation / config_.max_acceleration_innovation) *
        trust(acceleration_change / config_.acceleration_change_threshold) /
        config_.correction_time_constant;
      if (stationary_duration_ >= config_.stationary_recovery_delay) {
        // A direction gate alone can lock out correction after a posture error.
        // Reacquire slowly once acceleration is stable and the gyro is quiet.
        // Sustained linear acceleration is indistinguishable from tilt here.
        correction_rate = std::max(correction_rate,
          norm_trust / config_.stationary_recovery_time_constant);
      }
      const double weight = 1.0 - std::exp(-dt * correction_rate);
      const Eigen::Quaterniond correction = Eigen::Quaterniond::FromTwoVectors(up_, measured_up);
      up_ = (Eigen::Quaterniond::Identity().slerp(weight, correction) * up_).normalized();
    }
  }
  previous_acceleration_ = acceleration;
  return Eigen::Quaterniond::FromTwoVectors(up_, Eigen::Vector3d::UnitZ()).normalized();
}

namespace {
std::optional<Eigen::Quaterniond> interpolate_rotation(
  const std::deque<AttitudeSample> & history, std::int64_t stamp_ns,
  double max_sample_distance, Eigen::Quaterniond AttitudeSample::* member)
{
  if (history.empty()) {return std::nullopt;}
  const auto upper = std::lower_bound(history.begin(), history.end(), stamp_ns,
      [](const AttitudeSample & sample, std::int64_t time) {return sample.stamp_ns < time;});
  if (upper != history.end() && upper->stamp_ns == stamp_ns) {return (*upper).*member;}
  if (upper == history.begin() || upper == history.end()) {return std::nullopt;}
  const auto lower = std::prev(upper);
  if ((stamp_ns - lower->stamp_ns) * 1e-9 > max_sample_distance ||
    (upper->stamp_ns - stamp_ns) * 1e-9 > max_sample_distance)
  {
    return std::nullopt;
  }
  const double fraction = static_cast<double>(stamp_ns - lower->stamp_ns) /
    static_cast<double>(upper->stamp_ns - lower->stamp_ns);
  return ((*lower).*member).slerp(fraction, (*upper).*member).normalized();
}
}  // namespace

std::optional<Eigen::Quaterniond> interpolate_attitude(
  const std::deque<AttitudeSample> & history, std::int64_t stamp_ns,
  double max_sample_distance)
{
  return interpolate_rotation(history, stamp_ns, max_sample_distance, &AttitudeSample::rotation);
}

namespace {
bool host_is_bigendian() {
  const std::uint16_t value = 0x0102;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 1;
}
template<typename T>
T read_scalar(const std::uint8_t * data, bool swap) {
  std::uint8_t bytes[sizeof(T)];
  std::memcpy(bytes, data, sizeof(T));
  if (swap) {std::reverse(bytes, bytes + sizeof(T));}
  T value;
  std::memcpy(&value, bytes, sizeof(T));
  return value;
}
template<typename T>
void write_scalar(std::uint8_t * data, T value, bool swap) {
  std::uint8_t bytes[sizeof(T)];
  std::memcpy(bytes, &value, sizeof(T));
  if (swap) {std::reverse(bytes, bytes + sizeof(T));}
  std::memcpy(data, bytes, sizeof(T));
}
struct CloudLayout {
  const sensor_msgs::msg::PointField * xyz[3] = {nullptr, nullptr, nullptr};
  const sensor_msgs::msg::PointField * timestamp = nullptr;
  bool swap = false;
};

bool cloud_layout(
  const sensor_msgs::msg::PointCloud2 & input, bool require_timestamp,
  CloudLayout & layout, std::string & error)
{
  const char * names[3] = {"x", "y", "z"};
  for (const auto & field : input.fields) {
    for (int i = 0; i < 3; ++i) {
      if (field.name == names[i]) {layout.xyz[i] = &field;}
    }
    if (field.name == "timestamp") {layout.timestamp = &field;}
  }
  using Field = sensor_msgs::msg::PointField;
  for (const auto * field : layout.xyz) {
    if (!field || field->count != 1 ||
      (field->datatype != Field::FLOAT32 && field->datatype != Field::FLOAT64))
    {
      error = "x/y/z must each be a scalar FLOAT32 or FLOAT64 field";
      return false;
    }
    const std::size_t size = field->datatype == Field::FLOAT32 ? sizeof(float) : sizeof(double);
    if (static_cast<std::uint64_t>(field->offset) + size > input.point_step) {
      error = "Point field exceeds point_step";
      return false;
    }
  }
  if (static_cast<std::uint64_t>(input.width) * input.point_step > input.row_step ||
    static_cast<std::uint64_t>(input.height) * input.row_step > input.data.size())
  {
    error = "PointCloud2 data/row_step/point_step layout is inconsistent";
    return false;
  }
  if (require_timestamp) {
    if (!layout.timestamp || layout.timestamp->count != 1 ||
      layout.timestamp->datatype != Field::FLOAT64 ||
      static_cast<std::uint64_t>(layout.timestamp->offset) + sizeof(double) > input.point_step)
    {
      error = "timestamp must be a scalar FLOAT64 field in absolute nanoseconds";
      return false;
    }
    if (input.width == 0 || input.height == 0) {
      error = "Cannot choose a reference time for an empty cloud";
      return false;
    }
  }
  layout.swap = input.is_bigendian != host_is_bigendian();
  return true;
}

bool read_timestamp(
  const std::uint8_t * point, const CloudLayout & layout, std::int64_t & stamp,
  std::string & error)
{
  const double value = read_scalar<double>(point + layout.timestamp->offset, layout.swap);
  // INT64_MAX rounds to 2^63 as a double; use an exclusive 2^63 bound before casting.
  if (!std::isfinite(value) || value < 0 || value >= std::ldexp(1.0, 63)) {
    error = "Point timestamp is nonfinite, negative or outside the nanosecond range";
    return false;
  }
  stamp = static_cast<std::int64_t>(value);
  return true;
}

template<typename RotationAtPoint>
bool transform_cloud(
  const sensor_msgs::msg::PointCloud2 & input, const CloudLayout & layout,
  const std::string & output_frame, sensor_msgs::msg::PointCloud2 & output,
  RotationAtPoint rotation_at_point, std::string & error)
{
  using Field = sensor_msgs::msg::PointField;
  output = input;
  output.header.frame_id = output_frame;
  for (std::size_t row = 0; row < input.height; ++row) {
    for (std::size_t col = 0; col < input.width; ++col) {
      auto * point = output.data.data() + row * input.row_step + col * input.point_step;
      Eigen::Vector3d xyz;
      for (int i = 0; i < 3; ++i) {
        const auto * value = point + layout.xyz[i]->offset;
        xyz[i] = layout.xyz[i]->datatype == Field::FLOAT32 ?
          static_cast<double>(read_scalar<float>(value, layout.swap)) :
          read_scalar<double>(value, layout.swap);
      }
      if (!xyz.allFinite()) {continue;}
      Eigen::Matrix3d matrix;
      if (!rotation_at_point(point, matrix)) {return false;}
      const Eigen::Vector3d rotated = matrix * xyz;
      for (int i = 0; i < 3; ++i) {
        auto * value = point + layout.xyz[i]->offset;
        if (layout.xyz[i]->datatype == Field::FLOAT32) {
          write_scalar(value, static_cast<float>(rotated[i]), layout.swap);
        } else {write_scalar(value, rotated[i], layout.swap);}
      }
    }
  }
  error.clear();
  return true;
}
}  // namespace

bool rotate_cloud(
  const sensor_msgs::msg::PointCloud2 & input, const Eigen::Quaterniond & rotation,
  const std::string & output_frame, sensor_msgs::msg::PointCloud2 & output,
  std::string & error)
{
  CloudLayout layout;
  if (!cloud_layout(input, false, layout, error)) {return false;}
  if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-6) {
    error = "Invalid rotation";
    return false;
  }
  const Eigen::Matrix3d matrix = rotation.normalized().toRotationMatrix();
  return transform_cloud(input, layout, output_frame, output,
    [&matrix](const std::uint8_t *, Eigen::Matrix3d & at_point) {
      at_point = matrix;
      return true;
    }, error);
}

bool cloud_time_range(
  const sensor_msgs::msg::PointCloud2 & input, CloudTimeRange & times, std::string & error)
{
  CloudLayout layout;
  if (!cloud_layout(input, true, layout, error)) {return false;}
  times = {std::numeric_limits<std::int64_t>::max(), 0};
  for (std::size_t row = 0; row < input.height; ++row) {
    for (std::size_t col = 0; col < input.width; ++col) {
      const auto * point = input.data.data() + row * input.row_step + col * input.point_step;
      std::int64_t stamp;
      if (!read_timestamp(point, layout, stamp, error)) {return false;}
      times.start_ns = std::min(times.start_ns, stamp);
      times.end_ns = std::max(times.end_ns, stamp);
    }
  }
  if (times.end_ns / 1000000000 > std::numeric_limits<std::int32_t>::max()) {
    error = "Reference timestamp exceeds the ROS message seconds range";
    return false;
  }
  error.clear();
  return true;
}

bool deskew_cloud(
  const sensor_msgs::msg::PointCloud2 & input, const std::deque<AttitudeSample> & history,
  double max_sample_distance, const std::string & output_frame,
  sensor_msgs::msg::PointCloud2 & output, std::string & error)
{
  CloudTimeRange times;
  if (!cloud_time_range(input, times, error)) {return false;}
  const auto level = interpolate_attitude(history, times.end_ns, max_sample_distance);
  const auto reference = interpolate_rotation(
    history, times.end_ns, max_sample_distance, &AttitudeSample::gyro_rotation);
  if (!level || !reference) {
    error = "No matching IMU attitude at the scan reference time";
    return false;
  }
  const Eigen::Quaterniond to_level = (*level) * reference->conjugate();
  CloudLayout layout;
  if (!cloud_layout(input, true, layout, error)) {return false;}
  if (!transform_cloud(input, layout, output_frame, output,
      [&](const std::uint8_t * point, Eigen::Matrix3d & at_point) {
        std::int64_t stamp;
        if (!read_timestamp(point, layout, stamp, error)) {return false;}
        const auto gyro = interpolate_rotation(
          history, stamp, max_sample_distance, &AttitudeSample::gyro_rotation);
        if (!gyro) {
          error = "No matching IMU attitude for a point timestamp";
          return false;
        }
        at_point = (to_level * *gyro).normalized().toRotationMatrix();
        return true;
      }, error)) {return false;}
  output.header.stamp.sec = static_cast<std::int32_t>(times.end_ns / 1000000000);
  output.header.stamp.nanosec = static_cast<std::uint32_t>(times.end_ns % 1000000000);
  return true;
}

}  // namespace livox_mid360
