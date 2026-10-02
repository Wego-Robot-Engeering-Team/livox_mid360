#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>

class ImuVisualizer : public rclcpp::Node {
public:
  ImuVisualizer() : Node("imu_visualizer_node") {
    acceleration_scale_ = declare_parameter("acceleration_scale", 9.80665);
    acceleration_arrow_scale_ = declare_parameter("acceleration_arrow_scale", 0.1);
    gyro_arrow_scale_ = declare_parameter("gyro_arrow_scale", 0.5);
    const double rate = declare_parameter("publish_rate", 20.0);
    if (!std::isfinite(rate) || rate <= 0 || !std::isfinite(acceleration_scale_) ||
      acceleration_scale_ <= 0 || !std::isfinite(acceleration_arrow_scale_) ||
      acceleration_arrow_scale_ <= 0 || !std::isfinite(gyro_arrow_scale_) || gyro_arrow_scale_ <= 0)
    {throw std::invalid_argument("Invalid IMU visualization parameters");}
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("imu_markers", 10);
    sub_ = create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        latest_ = std::move(msg);
        latest_arrival_ = std::chrono::steady_clock::now();
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] {publish();});
  }

private:
  visualization_msgs::msg::Marker marker(int id, int type) const {
    visualization_msgs::msg::Marker result;
    result.header = latest_->header;
    result.ns = "mid360_imu";
    result.id = id;
    result.type = type;
    result.action = visualization_msgs::msg::Marker::ADD;
    result.pose.orientation.w = 1.0;
    result.color.a = 1.0;
    result.lifetime = rclcpp::Duration::from_seconds(0.3);
    return result;
  }
  void publish() {
    if (!latest_ || std::chrono::duration<double>(
        std::chrono::steady_clock::now() - latest_arrival_).count() > 0.3) {return;}
    const auto & a = latest_->linear_acceleration;
    const auto & g = latest_->angular_velocity;
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
      !std::isfinite(g.x) || !std::isfinite(g.y) || !std::isfinite(g.z)) {return;}
    visualization_msgs::msg::MarkerArray array;
    auto acceleration = marker(0, visualization_msgs::msg::Marker::ARROW);
    acceleration.color.g = 1.0;
    acceleration.scale.x = 0.035;
    acceleration.scale.y = 0.07;
    acceleration.scale.z = 0.1;
    geometry_msgs::msg::Point origin, endpoint;
    endpoint.x = a.x * acceleration_scale_ * acceleration_arrow_scale_;
    endpoint.y = a.y * acceleration_scale_ * acceleration_arrow_scale_;
    endpoint.z = a.z * acceleration_scale_ * acceleration_arrow_scale_;
    acceleration.points = {origin, endpoint};
    array.markers.push_back(acceleration);
    auto gyro = marker(1, visualization_msgs::msg::Marker::ARROW);
    gyro.color.r = 1.0;
    gyro.color.b = 0.3;
    gyro.scale = acceleration.scale;
    endpoint.x = g.x * gyro_arrow_scale_;
    endpoint.y = g.y * gyro_arrow_scale_;
    endpoint.z = g.z * gyro_arrow_scale_;
    gyro.points = {origin, endpoint};
    array.markers.push_back(gyro);
    auto text = marker(2, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    text.color.r = text.color.g = text.color.b = 1.0;
    text.pose.position.z = 1.2;
    text.scale.z = 0.12;
    std::ostringstream label;
    label << std::fixed << std::setprecision(2)
          << "acc [m/s^2]: " << a.x * acceleration_scale_ << ", "
          << a.y * acceleration_scale_ << ", " << a.z * acceleration_scale_
          << "\ngyro [rad/s]: " << g.x << ", " << g.y << ", " << g.z;
    text.text = label.str();
    array.markers.push_back(text);
    pub_->publish(array);
  }
  double acceleration_scale_, acceleration_arrow_scale_, gyro_arrow_scale_;
  sensor_msgs::msg::Imu::ConstSharedPtr latest_;
  std::chrono::steady_clock::time_point latest_arrival_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<ImuVisualizer>());}
  catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("imu_visualizer_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
