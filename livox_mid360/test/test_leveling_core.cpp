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

TEST(GravityFilter, RejectsLateralAccelerationPulseEvenWhenItsMagnitudeLooksLikeGravity) {
  auto config = fast_init();
  GravityFilter filter(config);
  const Eigen::Vector3d gravity(0, 0, config.gravity);
  const Eigen::Vector3d accelerated(3.0, 0, config.gravity);
  ASSERT_LT(std::abs(accelerated.norm() - config.gravity), config.acceleration_tolerance);
  ASSERT_TRUE(filter.update(0, gravity, Eigen::Vector3d::Zero()));
  for (int sample = 1; sample <= 80; ++sample) {  // A 400 ms horizontal acceleration pulse.
    const auto rotation = filter.update(sample * 5000000LL, accelerated, Eigen::Vector3d::Zero());
    ASSERT_TRUE(rotation);
    EXPECT_NEAR(rotation->angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-10);
  }
  for (int sample = 81; sample <= 180; ++sample) {
    const auto rotation = filter.update(sample * 5000000LL, gravity, Eigen::Vector3d::Zero());
    ASSERT_TRUE(rotation);
    EXPECT_NEAR(rotation->angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-10);
  }
}

TEST(GravityFilter, FollowsFastRotationWithoutMistakingRotatingGravityForAcceleration) {
  auto config = fast_init();
  GravityFilter filter(config);
  ASSERT_TRUE(filter.update(0, {0, 0, config.gravity}, Eigen::Vector3d::Zero()));
  for (int sample = 1; sample <= 60; ++sample) {
    const Eigen::Quaterniond expected(Eigen::AngleAxisd(sample * 0.03, Eigen::Vector3d::UnitX()));
    const Eigen::Vector3d acceleration = expected.conjugate() *
      (config.gravity * Eigen::Vector3d::UnitZ());
    const auto rotation = filter.update(sample * 5000000LL, acceleration, {6, 0, 0});
    ASSERT_TRUE(rotation);
    EXPECT_NEAR(rotation->angularDistance(expected), 0.0, 1e-9);
    EXPECT_NEAR(filter.gyro_rotation().angularDistance(expected), 0.0, 1e-9);
  }
}

TEST(GravityFilter, KeepsYawInGyroHistoryWithoutAddingYawToGravityLeveling) {
  auto config = fast_init();
  GravityFilter filter(config);
  ASSERT_TRUE(filter.update(0, {0, 0, config.gravity}, Eigen::Vector3d::Zero()));
  for (int sample = 1; sample <= 80; ++sample) {
    const auto rotation = filter.update(sample * 5000000LL, {0, 0, config.gravity}, {0, 0, 6});
    ASSERT_TRUE(rotation);
    EXPECT_NEAR(rotation->angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-10);
  }
  const Eigen::Quaterniond expected(Eigen::AngleAxisd(2.4, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(filter.gyro_rotation().angularDistance(expected), 0.0, 1e-9);
  filter.reset();
  EXPECT_NEAR(filter.gyro_rotation().angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-10);
}

TEST(GravityFilter, ReacquiresPersistentPostureErrorOnlyAfterStableDwell) {
  auto config = fast_init();
  GravityFilter filter(config);
  ASSERT_TRUE(filter.update(0, {0, 0, config.gravity}, Eigen::Vector3d::Zero()));
  // Unobserved acceleration permits an erroneous gyro-only tilt to accumulate.
  std::optional<Eigen::Quaterniond> rotation;
  for (int sample = 1; sample <= 60; ++sample) {
    rotation = filter.update(sample * 5000000LL, {0, 0, 20}, {1, 0, 0});
    ASSERT_TRUE(rotation);
  }
  ASSERT_NEAR(rotation->angularDistance(Eigen::Quaterniond::Identity()), 0.3, 1e-10);
  for (int sample = 61; sample <= 160; ++sample) {
    rotation = filter.update(sample * 5000000LL, {0, 0, config.gravity}, Eigen::Vector3d::Zero());
    ASSERT_TRUE(rotation);
    EXPECT_NEAR(rotation->angularDistance(Eigen::Quaterniond::Identity()), 0.3, 1e-10);
  }
  for (int sample = 161; sample <= 1860; ++sample) {
    rotation = filter.update(sample * 5000000LL, {0, 0, config.gravity}, Eigen::Vector3d::Zero());
    ASSERT_TRUE(rotation);
  }
  EXPECT_LT(rotation->angularDistance(Eigen::Quaterniond::Identity()), 0.01);
  // A reset must clear the stationary dwell, not just the output attitude.
  filter.reset();
  ASSERT_TRUE(filter.update(0, {0, 0, config.gravity}, Eigen::Vector3d::Zero()));
  for (int sample = 1; sample <= 80; ++sample) {
    rotation = filter.update(sample * 5000000LL, {3, 0, config.gravity}, Eigen::Vector3d::Zero());
    ASSERT_TRUE(rotation);
    EXPECT_NEAR(rotation->angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-10);
  }
}

TEST(GravityFilter, ReacquiresStationaryPostureDespiteIntermittentAccelerometerNoise) {
  auto config = fast_init();
  GravityFilter filter(config);
  ASSERT_TRUE(filter.update(0, {0, 0, config.gravity}, Eigen::Vector3d::Zero()));
  std::optional<Eigen::Quaterniond> rotation;
  for (int sample = 1; sample <= 60; ++sample) {
    rotation = filter.update(sample * 5000000LL, {0, 0, 20}, {1, 0, 0});
    ASSERT_TRUE(rotation);
  }
  // Spikes exceed the old per-sample 0.2 m/s^2 stationarity threshold every
  // 25 ms; their mean compensated change remains below that threshold.
  for (int sample = 61; sample <= 1860; ++sample) {
    const double noise = sample % 5 == 0 ? (sample % 10 == 0 ? 0.35 : -0.35) : 0.0;
    rotation = filter.update(sample * 5000000LL, {noise, 0, config.gravity},
      Eigen::Vector3d::Zero());
    ASSERT_TRUE(rotation);
  }
  EXPECT_LT(rotation->angularDistance(Eigen::Quaterniond::Identity()), 0.015);
}

TEST(GravityFilter, RejectsAccelerationPulseAfterStationaryRecoveryDwellHasCompleted) {
  auto config = fast_init();
  GravityFilter filter(config);
  ASSERT_TRUE(filter.update(0, {0, 0, config.gravity}, Eigen::Vector3d::Zero()));
  for (int sample = 1; sample <= 400; ++sample) {
    ASSERT_TRUE(filter.update(sample * 5000000LL, {0, 0, config.gravity}, Eigen::Vector3d::Zero()));
  }
  // The first abrupt sample must invalidate the already-completed dwell,
  // even though the low-pass change estimate has not risen yet.
  for (int sample = 401; sample <= 480; ++sample) {
    const auto rotation = filter.update(sample * 5000000LL, {3, 0, config.gravity},
      Eigen::Vector3d::Zero());
    ASSERT_TRUE(rotation);
    EXPECT_NEAR(rotation->angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-10);
  }
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

TEST(GravityFilter, RejectsInvalidAdaptiveCorrectionParameters) {
  double FilterConfig::* const parameters[] = {
    &FilterConfig::max_acceleration_innovation, &FilterConfig::acceleration_change_threshold,
    &FilterConfig::stationary_recovery_delay, &FilterConfig::stationary_recovery_time_constant};
  for (const auto parameter : parameters) {
    for (const double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()})
    {
      auto config = fast_init();
      config.*parameter = invalid;
      EXPECT_THROW(GravityFilter{config}, std::invalid_argument);
    }
  }
  auto config = fast_init();
  config.max_acceleration_innovation = M_PI;
  EXPECT_THROW(GravityFilter{config}, std::invalid_argument);
}

TEST(Deskew, AlignsYawMotionToMaximumPointTimeAndPreservesVendorDataAndPadding) {
  auto input = make_cloud();
  const std::int64_t base = 42000000000LL;
  const std::size_t offsets[] = {0, 28, 64, 92};
  const std::int64_t elapsed[] = {0, 20000000, 5000000, 10000000};
  for (int point = 0; point < 4; ++point) {
    const double yaw = elapsed[point] * 1e-9 * 20.0;
    put(input.data, offsets[point], static_cast<float>(std::cos(yaw)));
    put(input.data, offsets[point] + 4, static_cast<float>(-std::sin(yaw)));
    put(input.data, offsets[point] + 18, static_cast<double>(base + elapsed[point]));
  }
  std::deque<AttitudeSample> history;
  for (const std::int64_t elapsed_ns : {0LL, 10000000LL, 20000000LL}) {
    history.push_back({base + elapsed_ns, Eigen::Quaterniond::Identity(),
      Eigen::Quaterniond(Eigen::AngleAxisd(elapsed_ns * 1e-9 * 20.0, Eigen::Vector3d::UnitZ()))});
  }
  CloudTimeRange times;
  Cloud output;
  std::string error;
  ASSERT_TRUE(cloud_time_range(input, times, error));
  EXPECT_EQ(times.start_ns, base);
  EXPECT_EQ(times.end_ns, base + 20000000);
  ASSERT_TRUE(deskew_cloud(input, history, 0.03, "livox_level", output, error)) << error;
  EXPECT_EQ(output.header.stamp.sec, 42);
  EXPECT_EQ(output.header.stamp.nanosec, 20000000u);
  EXPECT_EQ(output.header.frame_id, "livox_level");
  EXPECT_EQ(output.fields, input.fields);
  EXPECT_EQ(output.row_step, input.row_step);
  for (const auto offset : offsets) {
    EXPECT_NEAR(get<float>(output.data, offset), std::cos(0.4), 1e-6);
    EXPECT_NEAR(get<float>(output.data, offset + 4), -std::sin(0.4), 1e-6);
    EXPECT_EQ(std::vector<std::uint8_t>(output.data.begin() + offset + 12,
        output.data.begin() + offset + 28),
      std::vector<std::uint8_t>(input.data.begin() + offset + 12, input.data.begin() + offset + 28));
  }
  for (const std::size_t offset : {56u, 120u}) {
    EXPECT_EQ(std::vector<std::uint8_t>(output.data.begin() + offset, output.data.begin() + offset + 8),
      std::vector<std::uint8_t>(input.data.begin() + offset, input.data.begin() + offset + 8));
  }
}

TEST(Deskew, RemovesCombinedPitchAndYawDistortionWhileKeepingGroundHeight) {
  auto input = make_cloud();
  const std::int64_t base = 42000000000LL;
  const Eigen::Vector3d world_point(3, 2, -1);
  const double yaw[] = {-0.2, 0.2, 0.6};
  const double pitch[] = {-0.3, 0.1, 0.4};
  std::deque<AttitudeSample> history;
  for (int sample = 0; sample < 3; ++sample) {
    const Eigen::Quaterniond body_rotation = Eigen::Quaterniond(
      Eigen::AngleAxisd(yaw[sample], Eigen::Vector3d::UnitZ())) *
      Eigen::Quaterniond(Eigen::AngleAxisd(pitch[sample], Eigen::Vector3d::UnitY()));
    history.push_back({base + sample * 10000000LL,
      Eigen::Quaterniond(Eigen::AngleAxisd(pitch[sample], Eigen::Vector3d::UnitY())), body_rotation});
  }
  const std::size_t offsets[] = {0, 28, 64, 92};
  const int sample_indices[] = {0, 2, 1, 0};
  for (int point = 0; point < 4; ++point) {
    const auto & attitude = history[sample_indices[point]];
    const Eigen::Vector3d raw = attitude.gyro_rotation.conjugate() * world_point;
    for (int axis = 0; axis < 3; ++axis) {
      put(input.data, offsets[point] + axis * 4, static_cast<float>(raw[axis]));
    }
    put(input.data, offsets[point] + 18, static_cast<double>(attitude.stamp_ns));
  }
  Cloud output;
  std::string error;
  ASSERT_TRUE(deskew_cloud(input, history, 0.03, "livox_level", output, error)) << error;
  const Eigen::Vector3d expected = Eigen::AngleAxisd(-0.6, Eigen::Vector3d::UnitZ()) * world_point;
  for (const auto offset : offsets) {
    for (int axis = 0; axis < 3; ++axis) {
      EXPECT_NEAR(get<float>(output.data, offset + axis * 4), expected[axis], 1e-6);
    }
  }
}

TEST(Deskew, StaticTiltAgreesWithSingleRotationLeveling) {
  const auto input = make_cloud();
  const Eigen::Quaterniond tilt(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()));
  const std::deque<AttitudeSample> history = {{42000000000LL, tilt}};
  Cloud expected, output;
  std::string error;
  ASSERT_TRUE(rotate_cloud(input, tilt, "level", expected, error));
  ASSERT_TRUE(deskew_cloud(input, history, 0.03, "level", output, error));
  EXPECT_EQ(output.header, expected.header);
  EXPECT_EQ(output.data, expected.data);
}

TEST(Deskew, RejectsInteriorImuGapDespiteAvailableScanEndpoints) {
  auto input = make_cloud();
  put(input.data, 18, 42000000000.0);
  put(input.data, 28 + 18, 42100000000.0);
  put(input.data, 64 + 18, 42050000000.0);
  put(input.data, 92 + 18, 42000000000.0);
  const std::deque<AttitudeSample> history = {
    {42000000000LL, Eigen::Quaterniond::Identity()},
    {42100000000LL, Eigen::Quaterniond::Identity()}};
  Cloud output;
  std::string error;
  EXPECT_FALSE(deskew_cloud(input, history, 0.03, "level", output, error));
  EXPECT_FALSE(error.empty());
}

TEST(Deskew, RejectsMissingMalformedAndOutOfRangePointTimestamps) {
  CloudTimeRange times;
  std::string error;
  auto input = make_cloud();
  input.fields.pop_back();
  EXPECT_FALSE(cloud_time_range(input, times, error));
  input = make_cloud();
  input.fields.back().datatype = Field::FLOAT32;
  EXPECT_FALSE(cloud_time_range(input, times, error));
  for (const double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), std::ldexp(1.0, 63),
      (static_cast<double>(std::numeric_limits<std::int32_t>::max()) + 1.0) * 1e9})
  {
    input = make_cloud();
    put(input.data, 18, invalid);
    EXPECT_FALSE(cloud_time_range(input, times, error));
  }
}

TEST(Deskew, ReadsBigEndianUnalignedTimesAndPreservesInvalidPointsAndAuxiliaryBytes) {
  auto input = make_cloud();
  input.is_bigendian = true;
  put(input.data, 0, std::numeric_limits<float>::quiet_NaN());
  for (const std::size_t offset : {0u, 28u, 64u, 92u}) {
    for (const std::size_t field_offset : {0u, 4u, 8u, 12u}) {
      std::reverse(input.data.begin() + offset + field_offset,
        input.data.begin() + offset + field_offset + 4);
    }
    std::reverse(input.data.begin() + offset + 18, input.data.begin() + offset + 26);
  }
  CloudTimeRange times;
  Cloud output;
  std::string error;
  ASSERT_TRUE(cloud_time_range(input, times, error));
  EXPECT_EQ(times.start_ns, 42000000000LL);
  const Eigen::Quaterniond tilt(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  const std::deque<AttitudeSample> history = {{42000000000LL, tilt}};
  ASSERT_TRUE(deskew_cloud(input, history, 0.03, "level", output, error)) << error;
  EXPECT_EQ(std::vector<std::uint8_t>(output.data.begin(), output.data.begin() + 28),
    std::vector<std::uint8_t>(input.data.begin(), input.data.begin() + 28));
  for (const std::size_t offset : {28u, 64u, 92u}) {
    EXPECT_EQ(std::vector<std::uint8_t>(output.data.begin() + offset + 12,
        output.data.begin() + offset + 28),
      std::vector<std::uint8_t>(input.data.begin() + offset + 12, input.data.begin() + offset + 28));
  }
  std::reverse(output.data.begin() + 28 + 4, output.data.begin() + 28 + 8);
  EXPECT_NEAR(get<float>(output.data, 28 + 4), 1.0f, 1e-6);
}
