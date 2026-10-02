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
    ready_ = true;
  } else {
    const Eigen::Vector3d delta = (angular_velocity - gyro_bias_) * dt;
    const double angle = delta.norm();
    if (angle > 1e-12) {
      // A fixed world direction expressed in a rotating sensor moves inversely.
      up_ = Eigen::AngleAxisd(-angle, delta / angle) * up_;
    }
    if (valid_gravity) {
      const double weight = 1.0 - std::exp(-dt / config_.correction_time_constant);
      const Eigen::Vector3d blended = (1.0 - weight) * up_ + weight * acceleration / norm;
      if (blended.norm() > 1e-6) {up_ = blended.normalized();}
    }
  }
  return Eigen::Quaterniond::FromTwoVectors(up_, Eigen::Vector3d::UnitZ()).normalized();
}

std::optional<Eigen::Quaterniond> interpolate_attitude(
  const std::deque<AttitudeSample> & history, std::int64_t stamp_ns,
  double max_sample_distance)
{
  if (history.empty()) {return std::nullopt;}
  const auto upper = std::lower_bound(history.begin(), history.end(), stamp_ns,
      [](const AttitudeSample & sample, std::int64_t time) {return sample.stamp_ns < time;});
  if (upper != history.end() && upper->stamp_ns == stamp_ns) {return upper->rotation;}
  if (upper == history.begin() || upper == history.end()) {return std::nullopt;}
  const auto lower = std::prev(upper);
  if ((stamp_ns - lower->stamp_ns) * 1e-9 > max_sample_distance ||
    (upper->stamp_ns - stamp_ns) * 1e-9 > max_sample_distance)
  {
    return std::nullopt;
  }
  const double fraction = static_cast<double>(stamp_ns - lower->stamp_ns) /
    static_cast<double>(upper->stamp_ns - lower->stamp_ns);
  return lower->rotation.slerp(fraction, upper->rotation).normalized();
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
}  // namespace

bool rotate_cloud(
  const sensor_msgs::msg::PointCloud2 & input, const Eigen::Quaterniond & rotation,
  const std::string & output_frame, sensor_msgs::msg::PointCloud2 & output,
  std::string & error)
{
  const sensor_msgs::msg::PointField * fields[3] = {nullptr, nullptr, nullptr};
  const char * names[3] = {"x", "y", "z"};
  for (const auto & field : input.fields) {
    for (int i = 0; i < 3; ++i) {
      if (field.name == names[i]) {fields[i] = &field;}
    }
  }
  using Field = sensor_msgs::msg::PointField;
  for (const auto * field : fields) {
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
  if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-6) {
    error = "Invalid rotation";
    return false;
  }
  output = input;
  output.header.frame_id = output_frame;
  const Eigen::Matrix3d matrix = rotation.normalized().toRotationMatrix();
  const bool swap = input.is_bigendian != host_is_bigendian();
  for (std::size_t row = 0; row < input.height; ++row) {
    for (std::size_t col = 0; col < input.width; ++col) {
      auto * point = output.data.data() + row * input.row_step + col * input.point_step;
      Eigen::Vector3d xyz;
      for (int i = 0; i < 3; ++i) {
        const auto * value = point + fields[i]->offset;
        xyz[i] = fields[i]->datatype == Field::FLOAT32 ?
          static_cast<double>(read_scalar<float>(value, swap)) : read_scalar<double>(value, swap);
      }
      if (!xyz.allFinite()) {continue;}
      const Eigen::Vector3d rotated = matrix * xyz;
      for (int i = 0; i < 3; ++i) {
        auto * value = point + fields[i]->offset;
        if (fields[i]->datatype == Field::FLOAT32) {
          write_scalar(value, static_cast<float>(rotated[i]), swap);
        } else {write_scalar(value, rotated[i], swap);}
      }
    }
  }
  error.clear();
  return true;
}

}  // namespace livox_mid360
