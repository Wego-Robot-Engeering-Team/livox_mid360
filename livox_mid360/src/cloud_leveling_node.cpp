#include "livox_mid360/leveling_core.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <utility>

namespace livox_mid360 {
using namespace std::chrono_literals;

class CloudLevelingNode : public rclcpp::Node {
public:
  CloudLevelingNode() : Node("cloud_leveling_node"), filter_(filter_config()) {
    sensor_frame_ = declare_parameter("sensor_frame", std::string("livox_frame"));
    level_frame_ = declare_parameter("level_frame", std::string("livox_level"));
    acceleration_scale_ = declare_parameter("acceleration_scale", 9.80665);
    history_seconds_ = declare_parameter("history_seconds", 2.0);
    max_sample_distance_ = declare_parameter("max_sample_distance", 0.03);
    cloud_wait_timeout_ = declare_parameter("cloud_wait_timeout", 0.2);
    const auto pending_count = declare_parameter("max_pending_clouds", 10);
    if (sensor_frame_.empty() || level_frame_.empty() || sensor_frame_ == level_frame_ ||
      !std::isfinite(acceleration_scale_) || acceleration_scale_ <= 0 ||
      !std::isfinite(history_seconds_) || history_seconds_ <= 0 ||
      !std::isfinite(max_sample_distance_) || max_sample_distance_ <= 0 ||
      !std::isfinite(cloud_wait_timeout_) || cloud_wait_timeout_ <= 0 || pending_count <= 0)
    {
      throw std::invalid_argument("Invalid leveling node parameters");
    }
    max_pending_clouds_ = static_cast<std::size_t>(pending_count);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "points_leveled", rclcpp::SensorDataQoS());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu_orientation", rclcpp::SensorDataQoS());
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS().keep_last(200),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {on_imu(*msg);});
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {on_cloud(std::move(msg));});
    timeout_timer_ = create_wall_timer(20ms, [this] {process_pending();});
    stats_timer_ = create_wall_timer(5s, [this] {
        RCLCPP_INFO(get_logger(), "clouds received=%lu leveled=%lu dropped=%lu pending=%zu imu_ready=%s",
          static_cast<unsigned long>(received_), static_cast<unsigned long>(published_),
          static_cast<unsigned long>(dropped_), pending_.size(), filter_.ready() ? "yes" : "no");
      });
    RCLCPP_INFO(get_logger(), "Hold sensor still for IMU initialization; leveling %s -> %s",
      sensor_frame_.c_str(), level_frame_.c_str());
  }

private:
  FilterConfig filter_config() {
    FilterConfig config;
    config.gravity = declare_parameter("gravity", config.gravity);
    config.acceleration_tolerance = declare_parameter(
      "acceleration_tolerance", config.acceleration_tolerance);
    config.correction_time_constant = declare_parameter(
      "correction_time_constant", config.correction_time_constant);
    config.max_imu_gap = declare_parameter("max_imu_gap", config.max_imu_gap);
    config.initialization_max_gyro = declare_parameter(
      "initialization_max_gyro", config.initialization_max_gyro);
    const auto samples = declare_parameter("initialization_samples", 50);
    if (samples <= 0) {throw std::invalid_argument("initialization_samples must be positive");}
    config.initialization_samples = static_cast<std::size_t>(samples);
    max_imu_gap_ = config.max_imu_gap;
    return config;
  }

  void on_imu(const sensor_msgs::msg::Imu & msg) {
    if (msg.header.frame_id != sensor_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "IMU frame '%s' must match sensor_frame '%s'", msg.header.frame_id.c_str(), sensor_frame_.c_str());
      return;
    }
    const auto stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
    if (last_imu_stamp_ && stamp == *last_imu_stamp_) {return;}
    if (last_imu_stamp_ &&
      (stamp < *last_imu_stamp_ || (stamp - *last_imu_stamp_) * 1e-9 > max_imu_gap_))
    {
      filter_.reset();
      history_.clear();
      dropped_ += pending_.size();
      pending_.clear();
      RCLCPP_WARN(get_logger(), "IMU time discontinuity; restarting initialization");
    }
    const Eigen::Vector3d acceleration(
      msg.linear_acceleration.x * acceleration_scale_,
      msg.linear_acceleration.y * acceleration_scale_,
      msg.linear_acceleration.z * acceleration_scale_);
    const Eigen::Vector3d gyro(
      msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    if (!acceleration.allFinite() || !gyro.allFinite()) {return;}
    last_imu_stamp_ = stamp;
    const auto rotation = filter_.update(stamp, acceleration, gyro);
    if (!rotation) {return;}
    history_.push_back({stamp, *rotation});
    while (history_.size() > 1 && (stamp - history_.front().stamp_ns) * 1e-9 > history_seconds_) {
      history_.pop_front();
    }
    auto orientation_msg = msg;
    orientation_msg.linear_acceleration.x = acceleration.x();
    orientation_msg.linear_acceleration.y = acceleration.y();
    orientation_msg.linear_acceleration.z = acceleration.z();
    orientation_msg.orientation.x = rotation->x();
    orientation_msg.orientation.y = rotation->y();
    orientation_msg.orientation.z = rotation->z();
    orientation_msg.orientation.w = rotation->w();
    // No statistical covariance estimate is available (all zeros = unknown).
    orientation_msg.orientation_covariance.fill(0.0);
    for (auto & covariance : orientation_msg.linear_acceleration_covariance) {
      covariance *= acceleration_scale_ * acceleration_scale_;
    }
    imu_pub_->publish(orientation_msg);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = msg.header.stamp;
    transform.header.frame_id = level_frame_;
    transform.child_frame_id = sensor_frame_;
    transform.transform.rotation = orientation_msg.orientation;
    tf_->sendTransform(transform);
    process_pending();
  }

  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    ++received_;
    if (msg->header.frame_id != sensor_frame_) {
      ++dropped_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Point cloud frame '%s' must match sensor_frame '%s'",
        msg->header.frame_id.c_str(), sensor_frame_.c_str());
      return;
    }
    if (pending_.size() >= max_pending_clouds_) {pending_.pop_front(); ++dropped_;}
    pending_.push_back({std::move(msg), std::chrono::steady_clock::now()});
    process_pending();
  }

  void process_pending() {
    const auto now = std::chrono::steady_clock::now();
    for (auto it = pending_.begin(); it != pending_.end();) {
      const auto stamp = rclcpp::Time(it->cloud->header.stamp).nanoseconds();
      const auto rotation = interpolate_attitude(history_, stamp, max_sample_distance_);
      const bool expired = std::chrono::duration<double>(now - it->arrival).count() > cloud_wait_timeout_;
      const bool too_old = !history_.empty() && stamp < history_.front().stamp_ns;
      // Once history brackets a stamp, a failed interpolation means an IMU gap.
      const bool gap = !history_.empty() && stamp < history_.back().stamp_ns && !rotation;
      if (rotation) {
        sensor_msgs::msg::PointCloud2 output;
        std::string error;
        if (rotate_cloud(*it->cloud, *rotation, level_frame_, output, error)) {
          cloud_pub_->publish(std::move(output));
          ++published_;
        } else {
          ++dropped_;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.c_str());
        }
        it = pending_.erase(it);
      } else if (expired || too_old || gap) {
        ++dropped_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Cloud dropped: no matching initialized IMU attitude; check timestamps and IMU stream");
        it = pending_.erase(it);
      } else {++it;}
    }
  }

  struct PendingCloud {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    std::chrono::steady_clock::time_point arrival;
  };
  double max_imu_gap_ = 0.1;
  GravityFilter filter_;
  std::string sensor_frame_, level_frame_;
  double acceleration_scale_, history_seconds_, max_sample_distance_, cloud_wait_timeout_;
  std::size_t max_pending_clouds_;
  std::optional<std::int64_t> last_imu_stamp_;
  std::deque<AttitudeSample> history_;
  std::deque<PendingCloud> pending_;
  std::uint64_t received_ = 0, published_ = 0, dropped_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timeout_timer_, stats_timer_;
};
}  // namespace livox_mid360

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<livox_mid360::CloudLevelingNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("cloud_leveling_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
