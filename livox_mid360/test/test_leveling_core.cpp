#include "livox_mid360/leveling_core.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>

using namespace livox_mid360;
using Cloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;

namespace {
Field field(const std::string & name, std::uint32_t offset, std::uint8_t datatype) {
  Field result;
  result.name = name;
  result.offset = offset;
  result.datatype = datatype;
  result.count = 1;
  return result;
}
template<typename T>
void put(std::vector<std::uint8_t> & bytes, std::size_t offset, T value) {
  std::memcpy(bytes.data() + offset, &value, sizeof(T));
}
template<typename T>
T get(const std::vector<std::uint8_t> & bytes, std::size_t offset) {
  T value;
  std::memcpy(&value, bytes.data() + offset, sizeof(T));
  return value;
}
Cloud make_cloud() {
  Cloud cloud;
  cloud.header.frame_id = "livox_frame";
  cloud.header.stamp.sec = 42;
  cloud.width = 2;
  cloud.height = 2;
  cloud.point_step = 28;
  cloud.row_step = 64;  // Organized cloud with eight padding bytes per row.
  cloud.fields = {field("x", 0, Field::FLOAT32), field("y", 4, Field::FLOAT32),
    field("z", 8, Field::FLOAT32), field("intensity", 12, Field::FLOAT32),
    field("tag", 16, Field::UINT8), field("timestamp", 18, Field::FLOAT64)};
  cloud.data.resize(128, 0xA5);
  for (const std::size_t offset : {0u, 28u, 64u, 92u}) {
    put(cloud.data, offset, 1.0f);
    put(cloud.data, offset + 4, 0.0f);
    put(cloud.data, offset + 8, 0.0f);
    put(cloud.data, offset + 12, 12.5f);
    put(cloud.data, offset + 18, 42000000000.0);
  }
  return cloud;
}
FilterConfig fast_init() {
  FilterConfig config;
  config.initialization_samples = 1;
  return config;
}
}  // namespace

TEST(GravityFilter, StaticTiltLevelsGroundWithoutChangingHeadingAboutGravity) {
  GravityFilter filter(fast_init());
  const Eigen::Quaterniond sensor_to_level(
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()));
  const Eigen::Vector3d acceleration = sensor_to_level.conjugate() *
    (9.80665 * Eigen::Vector3d::UnitZ());
  const auto rotation = filter.update(1000000000, acceleration, Eigen::Vector3d::Zero());
  ASSERT_TRUE(rotation);
  EXPECT_NEAR(((*rotation) * acceleration).x(), 0.0, 1e-10);
  EXPECT_NEAR(((*rotation) * acceleration).y(), 0.0, 1e-10);
  const Eigen::Vector3d ground_point(3.0, 2.0, -1.0);
  const Eigen::Vector3d raw = sensor_to_level.conjugate() * ground_point;
  EXPECT_NEAR(((*rotation) * raw).z(), -1.0, 1e-10);
  EXPECT_NEAR(rotation->z(), 0.0, 1e-10);
}

TEST(GravityFilter, InitializesOnlyWithConsecutiveStationarySamples) {
  auto config = fast_init();
  config.initialization_samples = 3;
  GravityFilter filter(config);
  const Eigen::Vector3d a(0, 0, config.gravity);
  EXPECT_FALSE(filter.update(0, a, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(filter.update(5000000, a, Eigen::Vector3d::UnitX()));
  EXPECT_FALSE(filter.update(10000000, a, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(filter.update(15000000, a, Eigen::Vector3d::Zero()));
  EXPECT_TRUE(filter.update(20000000, a, Eigen::Vector3d::Zero()));
}

TEST(GravityFilter, PropagatesGyroWithCorrectSignWhenAccelerationIsUntrusted) {
  GravityFilter filter(fast_init());
  ASSERT_TRUE(filter.update(0, {0, 0, 9.80665}, Eigen::Vector3d::Zero()));
  const auto rotation = filter.update(10000000, {0, 0, 20}, {1, 0, 0});
  ASSERT_TRUE(rotation);
  const auto expected = Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitX());
  EXPECT_NEAR(rotation->angularDistance(Eigen::Quaterniond(expected)), 0.0, 1e-10);
}

TEST(GravityFilter, CalibratesStationaryGyroBias) {
  GravityFilter filter(fast_init());
  const Eigen::Vector3d gyro(0.01, -0.02, 0.01);
  ASSERT_TRUE(filter.update(0, {0, 0, 9.80665}, gyro));
  const auto rotation = filter.update(10000000, {0, 0, 20}, gyro);
  ASSERT_TRUE(rotation);
  EXPECT_NEAR(rotation->angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-10);
}

TEST(GravityFilter, RejectsNaNsDuplicatesAndRestartsOnDiscontinuities) {
  auto config = fast_init();
  config.initialization_samples = 2;
  GravityFilter filter(config);
  const Eigen::Vector3d a(0, 0, config.gravity);
  EXPECT_FALSE(filter.update(10000000, a, Eigen::Vector3d::Zero()));
  ASSERT_TRUE(filter.update(15000000, a, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(filter.update(15000000, a, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(filter.update(20000000, {NAN, 0, 0}, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(filter.update(1000000000, a, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(filter.ready());
  ASSERT_TRUE(filter.update(1005000000, a, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(filter.update(0, a, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(filter.ready());
}

TEST(AttitudeHistory, InterpolatesAtCloudTimestampAndRejectsGapsAndExtrapolation) {
  std::deque<AttitudeSample> history = {{1000000000, Eigen::Quaterniond::Identity()},
    {1020000000, Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()))}};
  const auto q = interpolate_attitude(history, 1010000000, 0.03);
  ASSERT_TRUE(q);
  EXPECT_NEAR(q->angularDistance(Eigen::Quaterniond(
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()))), 0.0, 1e-10);
  EXPECT_FALSE(interpolate_attitude(history, 999000000, 0.03));
  EXPECT_FALSE(interpolate_attitude(history, 1030000000, 0.03));
  EXPECT_FALSE(interpolate_attitude(history, 1010000000, 0.005));
  EXPECT_TRUE(interpolate_attitude(history, 1000000000, 0.005));
  EXPECT_FALSE(interpolate_attitude({}, 0, 0.03));
}

TEST(PointCloud, RotatesOrganizedCloudAndPreservesAllOtherBytesAndStamp) {
  const Cloud input = make_cloud();
  Cloud output;
  std::string error;
  ASSERT_TRUE(rotate_cloud(input, Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())), "livox_level", output, error));
  EXPECT_EQ(output.header.frame_id, "livox_level");
  EXPECT_EQ(output.header.stamp, input.header.stamp);
  EXPECT_EQ(output.row_step, input.row_step);
  EXPECT_EQ(output.fields, input.fields);
  for (const std::size_t offset : {0u, 28u, 64u, 92u}) {
    EXPECT_NEAR(get<float>(output.data, offset), 0.0f, 1e-6);
    EXPECT_NEAR(get<float>(output.data, offset + 4), 1.0f, 1e-6);
    EXPECT_EQ(get<float>(output.data, offset + 12), 12.5f);
    EXPECT_EQ(get<double>(output.data, offset + 18), 42000000000.0);
    EXPECT_EQ(std::vector<std::uint8_t>(input.data.begin() + offset + 12, input.data.begin() + offset + 28),
      std::vector<std::uint8_t>(output.data.begin() + offset + 12, output.data.begin() + offset + 28));
  }
  EXPECT_EQ(std::vector<std::uint8_t>(input.data.begin() + 56, input.data.begin() + 64),
    std::vector<std::uint8_t>(output.data.begin() + 56, output.data.begin() + 64));
  EXPECT_EQ(get<float>(input.data, 0), 1.0f);
}

TEST(PointCloud, PreservesInvalidPointsWithoutRewritingNaNs) {
  auto input = make_cloud();
  put(input.data, 0, std::numeric_limits<float>::quiet_NaN());
  Cloud output;
  std::string error;
  ASSERT_TRUE(rotate_cloud(input, Eigen::Quaterniond::Identity(), "level", output, error));
  EXPECT_EQ(std::vector<std::uint8_t>(input.data.begin(), input.data.begin() + 28),
    std::vector<std::uint8_t>(output.data.begin(), output.data.begin() + 28));
}

TEST(PointCloud, SupportsBigEndianUnalignedFields) {
  auto input = make_cloud();
  input.is_bigendian = true;
  for (const std::size_t offset : {0u, 28u, 64u, 92u}) {
    for (const std::size_t field_offset : {0u, 4u, 8u}) {
      std::reverse(input.data.begin() + offset + field_offset,
        input.data.begin() + offset + field_offset + 4);
    }
  }
  Cloud output;
  std::string error;
  ASSERT_TRUE(rotate_cloud(input, Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())), "level", output, error));
  std::reverse(output.data.begin() + 4, output.data.begin() + 8);
  EXPECT_NEAR(get<float>(output.data, 4), 1.0f, 1e-6);
}

TEST(PointCloud, SupportsFloat64Coordinates) {
  Cloud input;
  input.width = input.height = 1;
  input.point_step = input.row_step = 24;
  input.fields = {field("x", 0, Field::FLOAT64), field("y", 8, Field::FLOAT64), field("z", 16, Field::FLOAT64)};
  input.data.resize(24);
  put(input.data, 0, 3.0);
  put(input.data, 8, 4.0);
  put(input.data, 16, 5.0);
  Cloud output;
  std::string error;
  ASSERT_TRUE(rotate_cloud(input, Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())), "level", output, error));
  EXPECT_NEAR(get<double>(output.data, 0), -4.0, 1e-10);
  EXPECT_NEAR(get<double>(output.data, 8), 3.0, 1e-10);
}

TEST(PointCloud, RejectsMalformedLayouts) {
  auto input = make_cloud();
  Cloud output;
  std::string error;
  input.data.resize(5);
  EXPECT_FALSE(rotate_cloud(input, Eigen::Quaterniond::Identity(), "level", output, error));
  input = make_cloud();
  input.fields[0].offset = 27;
  EXPECT_FALSE(rotate_cloud(input, Eigen::Quaterniond::Identity(), "level", output, error));
  input = make_cloud();
  input.fields[0].datatype = Field::UINT32;
  EXPECT_FALSE(rotate_cloud(input, Eigen::Quaterniond::Identity(), "level", output, error));
  input = make_cloud();
  input.fields.erase(input.fields.begin());
  EXPECT_FALSE(rotate_cloud(input, Eigen::Quaterniond::Identity(), "level", output, error));
}

TEST(GravityFilter, RejectsInvalidConfiguration) {
  auto config = fast_init();
  config.correction_time_constant = 0;
  EXPECT_THROW(GravityFilter{config}, std::invalid_argument);
}
