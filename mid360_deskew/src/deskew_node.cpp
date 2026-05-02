#include "mid360_deskew/deskew_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <type_traits>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace mid360_deskew
{

namespace
{
Eigen::Isometry3d transformToEigen(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & translation = transform.transform.translation;
  const auto & rotation = transform.transform.rotation;

  Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
  if (q.norm() < std::numeric_limits<double>::epsilon()) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }

  Eigen::Isometry3d eigen = Eigen::Isometry3d::Identity();
  eigen.linear() = q.toRotationMatrix();
  eigen.translation() = Eigen::Vector3d(translation.x, translation.y, translation.z);
  return eigen;
}

rclcpp::Duration secondsToDuration(double seconds)
{
  return rclcpp::Duration::from_seconds(seconds);
}

builtin_interfaces::msg::Time timeToMsg(const rclcpp::Time & time)
{
  builtin_interfaces::msg::Time msg;
  const std::int64_t ns = time.nanoseconds();
  std::int64_t sec = ns / 1000000000LL;
  std::int64_t nsec = ns % 1000000000LL;
  if (nsec < 0) {
    nsec += 1000000000LL;
    --sec;
  }
  msg.sec = static_cast<std::int32_t>(sec);
  msg.nanosec = static_cast<std::uint32_t>(nsec);
  return msg;
}

double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

Pose interpolatePose(const Pose & start, const Pose & end, double alpha, const rclcpp::Time & stamp)
{
  Pose pose;
  pose.stamp = stamp;
  pose.position = start.position + alpha * (end.position - start.position);
  pose.orientation = start.orientation.slerp(alpha, end.orientation);
  pose.orientation.normalize();
  return pose;
}

Eigen::Isometry3d rotationOnlyPose(const Eigen::Quaterniond & orientation)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = orientation.normalized().toRotationMatrix();
  return pose;
}

template<typename PointT, typename = void>
struct HasLivoxTag : std::false_type {};

template<typename PointT>
struct HasLivoxTag<PointT, std::void_t<decltype(std::declval<PointT>().tag)>> : std::true_type {};

template<typename PointT, typename = void>
struct HasLivoxLine : std::false_type {};

template<typename PointT>
struct HasLivoxLine<PointT, std::void_t<decltype(std::declval<PointT>().line)>> : std::true_type {};

template<typename PointT>
std::uint8_t livoxTagOrZero(const PointT & point)
{
  if constexpr (HasLivoxTag<PointT>::value) {
    return point.tag;
  }
  return 0U;
}

template<typename PointT>
std::uint8_t livoxLineOrZero(const PointT & point)
{
  if constexpr (HasLivoxLine<PointT>::value) {
    return point.line;
  }
  return 0U;
}

template<typename T>
void writePointField(std::vector<std::uint8_t> & data, std::size_t offset, const T & value)
{
  std::memcpy(data.data() + offset, &value, sizeof(T));
}
}  // namespace

DeskewNode::DeskewNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mid360_deskew_node", options),
  pose_buffer_(5.0, 0.05, false),
  imu_buffer_(5.0, 0.02)
{
  cloud_input_type_ = declare_parameter<std::string>("cloud_input_type", "pointcloud2");
  input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/mid360/points");
  custom_cloud_topic_ = declare_parameter<std::string>("custom_cloud_topic", "/livox/lidar");
  output_cloud_topic_ = declare_parameter<std::string>("output_cloud_topic", "/mid360/points_deskewed");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/mid360/imu");

  fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", "mid360_link");
  imu_frame_ = declare_parameter<std::string>("imu_frame", "mid360_imu_link");

  deskew_method_ = declare_parameter<std::string>("deskew_method", "odom");
  fallback_method_ = declare_parameter<std::string>("fallback_method", "none");
  deskew_target_time_ = declare_parameter<std::string>("deskew_target_time", "scan_end");
  rotation_source_ = declare_parameter<std::string>("rotation_source", "imu");
  translation_source_ = declare_parameter<std::string>("translation_source", "odom");
  point_time_field_ = declare_parameter<std::string>("point_time_field", "offset_time");
  point_time_unit_ = declare_parameter<std::string>("point_time_unit", "nanosecond");
  cloud_stamp_type_ = declare_parameter<std::string>("cloud_stamp_type", "scan_start");
  custom_msg_timebase_type_ = declare_parameter<std::string>("custom_msg_timebase_type", "ros_header");
  custom_msg_offset_unit_ = declare_parameter<std::string>("custom_msg_offset_unit", "nanosecond");
  custom_msg_frame_id_ = declare_parameter<std::string>("custom_msg_frame_id", lidar_frame_);

  cloud_time_offset_ = declare_parameter<double>("cloud_time_offset", 0.0);
  odom_time_offset_config_ = declare_parameter<double>("odom_time_offset", 0.0);
  imu_time_offset_config_ = declare_parameter<double>("imu_time_offset", 0.0);

  pose_buffer_duration_ = declare_parameter<double>("pose_buffer_duration", 5.0);
  max_allowed_time_gap_ = declare_parameter<double>("max_allowed_time_gap", 0.05);
  allow_extrapolation_ = declare_parameter<bool>("allow_extrapolation", false);
  imu_buffer_duration_ = declare_parameter<double>("imu_buffer_duration", 5.0);
  max_allowed_imu_gap_ = declare_parameter<double>("max_allowed_imu_gap", 0.02);
  estimate_gyro_bias_ = declare_parameter<bool>("estimate_gyro_bias", true);
  gyro_bias_estimation_duration_ = declare_parameter<double>("gyro_bias_estimation_duration", 2.0);
  gyro_bias_static_threshold_ = declare_parameter<double>("gyro_bias_static_threshold", 0.05);
  use_acc_for_translation_ = declare_parameter<bool>("use_acc_for_translation", false);
  imu_rotation_sign_ = declare_parameter<double>("imu_rotation_sign", 1.0);

  min_range_ = declare_parameter<double>("min_range", 0.05);
  max_range_ = declare_parameter<double>("max_range", 5.0);
  remove_nan_ = declare_parameter<bool>("remove_nan", true);
  filter_mode_ = declare_parameter<std::string>("filter_mode", "keep_size_nan");

  drop_if_no_point_time_ = declare_parameter<bool>("drop_if_no_point_time", true);
  drop_if_no_pose_ = declare_parameter<bool>("drop_if_no_pose", true);
  drop_if_no_imu_ = declare_parameter<bool>("drop_if_no_imu", false);
  publish_raw_when_failed_ = declare_parameter<bool>("publish_raw_when_failed", false);

  enable_time_check_ = declare_parameter<bool>("enable_time_check", true);
  time_check_window_ = declare_parameter<double>("time_check_window", 3.0);
  time_check_min_samples_ = declare_parameter<int>("time_check_min_samples", 20);
  apply_auto_time_offset_ = declare_parameter<bool>("apply_auto_time_offset", false);
  auto_time_offset_max_abs_ = declare_parameter<double>("auto_time_offset_max_abs", 0.1);
  delay_jitter_warning_threshold_ = declare_parameter<double>("delay_jitter_warning_threshold", 0.01);
  print_time_check_result_ = declare_parameter<bool>("print_time_check_result", true);

  enable_debug_log_ = declare_parameter<bool>("enable_debug_log", true);

  if (!isSupportedCloudInputType(cloud_input_type_)) {
    RCLCPP_FATAL(
      get_logger(),
      "Unsupported cloud_input_type '%s'. Use 'pointcloud2' or 'custom_msg'.",
      cloud_input_type_.c_str());
    throw std::runtime_error("unsupported cloud_input_type: " + cloud_input_type_);
  }

  if (!isSupportedDeskewMethod(deskew_method_)) {
    RCLCPP_WARN(
      get_logger(),
      "Unsupported deskew_method '%s'. Falling back to 'odom'.",
      deskew_method_.c_str());
    deskew_method_ = "odom";
  }

  if (!isSupportedFallbackMethod(fallback_method_)) {
    RCLCPP_WARN(
      get_logger(),
      "Unsupported fallback_method '%s'. Falling back to 'none'.",
      fallback_method_.c_str());
    fallback_method_ = "none";
  }

  if (!isSupportedRotationSource(rotation_source_)) {
    RCLCPP_WARN(
      get_logger(),
      "Unsupported rotation_source '%s'. Falling back to 'imu'.",
      rotation_source_.c_str());
    rotation_source_ = "imu";
  }

  if (translation_source_ != "odom") {
    RCLCPP_WARN(
      get_logger(),
      "Only translation_source='odom' is implemented. Requested '%s' will be treated as odom.",
      translation_source_.c_str());
    translation_source_ = "odom";
  }

  if (!isSupportedScanStampType(cloud_stamp_type_)) {
    RCLCPP_WARN(
      get_logger(),
      "Unsupported cloud_stamp_type '%s'. Falling back to 'scan_start'.",
      cloud_stamp_type_.c_str());
    cloud_stamp_type_ = "scan_start";
  }

  if (!isSupportedDeskewTargetTime(deskew_target_time_)) {
    RCLCPP_WARN(
      get_logger(),
      "Unsupported deskew_target_time '%s'. Falling back to 'scan_end'.",
      deskew_target_time_.c_str());
    deskew_target_time_ = "scan_end";
  }

  if (filter_mode_ != "keep_size_nan" && filter_mode_ != "compact") {
    RCLCPP_WARN(
      get_logger(),
      "Unsupported filter_mode '%s'. Falling back to 'keep_size_nan'.",
      filter_mode_.c_str());
    filter_mode_ = "keep_size_nan";
  }

  if (cloud_input_type_ == "pointcloud2" && filter_mode_ == "compact") {
    RCLCPP_WARN(
      get_logger(),
      "filter_mode='compact' is only applied by the CustomMsg input path. PointCloud2 output keeps the original cloud layout.");
  }

  if (!isSupportedCustomMsgTimebaseType(custom_msg_timebase_type_)) {
    RCLCPP_WARN(
      get_logger(),
      "Unsupported custom_msg_timebase_type '%s'. Falling back to 'ros_header'.",
      custom_msg_timebase_type_.c_str());
    custom_msg_timebase_type_ = "ros_header";
  }

  double custom_msg_offset_scale = 0.0;
  if (!customMsgOffsetUnitScale(custom_msg_offset_unit_, custom_msg_offset_scale)) {
    RCLCPP_WARN(
      get_logger(),
      "Unsupported custom_msg_offset_unit '%s'. Falling back to 'nanosecond'.",
      custom_msg_offset_unit_.c_str());
    custom_msg_offset_unit_ = "nanosecond";
  }

  if (drop_if_no_pose_ && publish_raw_when_failed_) {
    RCLCPP_WARN(
      get_logger(),
      "Both drop_if_no_pose and publish_raw_when_failed are true. drop_if_no_pose takes precedence.");
  }

  if (use_acc_for_translation_) {
    RCLCPP_WARN(
      get_logger(),
      "use_acc_for_translation=true was requested, but this first version does not integrate IMU acceleration for translation. Odom remains the translation source.");
    use_acc_for_translation_ = false;
  }

  if (std::abs(imu_rotation_sign_) < 0.5) {
    RCLCPP_WARN(
      get_logger(),
      "imu_rotation_sign=%.3f is invalid. Falling back to 1.0. Use 1.0 or -1.0 for direction validation.",
      imu_rotation_sign_);
    imu_rotation_sign_ = 1.0;
  } else {
    imu_rotation_sign_ = imu_rotation_sign_ < 0.0 ? -1.0 : 1.0;
  }

  pose_buffer_.configure(pose_buffer_duration_, max_allowed_time_gap_, allow_extrapolation_);
  imu_buffer_.configure(imu_buffer_duration_, max_allowed_imu_gap_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_cloud_topic_,
    rclcpp::SensorDataQoS());

  if (cloud_input_type_ == "pointcloud2") {
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&DeskewNode::cloudCallback, this, std::placeholders::_1));
  } else {
    custom_cloud_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      custom_cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&DeskewNode::customMsgCallback, this, std::placeholders::_1));
  }

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_,
    rclcpp::QoS(100),
    std::bind(&DeskewNode::odomCallback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&DeskewNode::imuCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "mid360_deskew_node started. cloud_input_type=%s pointcloud2_input=%s custom_input=%s output=%s odom=%s imu=%s fixed_frame=%s base_frame=%s lidar_frame=%s imu_frame=%s deskew_method=%s fallback_method=%s rotation_source=%s translation_source=%s",
    cloud_input_type_.c_str(),
    input_cloud_topic_.c_str(),
    custom_cloud_topic_.c_str(),
    output_cloud_topic_.c_str(),
    odom_topic_.c_str(),
    imu_topic_.c_str(),
    fixed_frame_.c_str(),
    base_frame_.c_str(),
    lidar_frame_.c_str(),
    imu_frame_.c_str(),
    deskew_method_.c_str(),
    fallback_method_.c_str(),
    rotation_source_.c_str(),
    translation_source_.c_str());
}

void DeskewNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (enable_time_check_) {
    recordOdomDelay(*msg);
  }

  if (!msg->header.frame_id.empty() && msg->header.frame_id != fixed_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Odometry header.frame_id is '%s', expected fixed_frame '%s'. The node assumes odom pose is T_%s_%s.",
      msg->header.frame_id.c_str(),
      fixed_frame_.c_str(),
      fixed_frame_.c_str(),
      base_frame_.c_str());
  }

  if (!msg->child_frame_id.empty() && msg->child_frame_id != base_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Odometry child_frame_id is '%s', expected '%s'. The node assumes odom pose is T_%s_%s.",
      msg->child_frame_id.c_str(),
      base_frame_.c_str(),
      fixed_frame_.c_str(),
      base_frame_.c_str());
  }

  pose_buffer_.addOdometry(*msg);
}

void DeskewNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (enable_time_check_) {
    recordImuDelay(*msg);
  }

  Eigen::Vector3d angular_velocity_imu(
    msg->angular_velocity.x,
    msg->angular_velocity.y,
    msg->angular_velocity.z);
  Eigen::Vector3d linear_acceleration_imu(
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z);

  const std::string source_imu_frame = msg->header.frame_id.empty() ? imu_frame_ : msg->header.frame_id;
  Eigen::Vector3d angular_velocity_lidar = angular_velocity_imu;
  Eigen::Vector3d linear_acceleration_lidar = linear_acceleration_imu;

  // IMU 角速度必须先转到 lidar_frame，下游积分才表示 MID360 点云坐标系内的相对旋转。
  if (source_imu_frame != lidar_frame_) {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        lidar_frame_,
        source_imu_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(0.01));
      const Eigen::Isometry3d t_lidar_imu = transformToEigen(transform);
      angular_velocity_lidar = t_lidar_imu.linear() * angular_velocity_imu;
      linear_acceleration_lidar = t_lidar_imu.linear() * linear_acceleration_imu;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Failed to transform IMU from frame '%s' to lidar_frame '%s': %s",
        source_imu_frame.c_str(),
        lidar_frame_.c_str(),
        ex.what());
      return;
    }
  }

  const rclcpp::Time stamp(msg->header.stamp);
  // 启动初期只用静止 gyro 样本估计零偏；估计完成后清空旧 buffer，避免混用未扣 bias 的早期样本。
  if (estimate_gyro_bias_ && !gyro_bias_estimation_finished_) {
    if (!gyro_bias_window_started_) {
      gyro_bias_window_started_ = true;
      gyro_bias_window_start_ = stamp;
    }

    const double elapsed = (stamp - gyro_bias_window_start_).seconds();
    if (elapsed <= gyro_bias_estimation_duration_) {
      if (angular_velocity_lidar.norm() < gyro_bias_static_threshold_) {
        gyro_bias_sum_ += angular_velocity_lidar;
        ++gyro_bias_sample_count_;
      }
    } else {
      gyro_bias_estimation_finished_ = true;
      if (gyro_bias_sample_count_ > 0) {
        gyro_bias_ = gyro_bias_sum_ / static_cast<double>(gyro_bias_sample_count_);
        gyro_bias_estimated_ = true;
        imu_buffer_.clear();
        RCLCPP_INFO(
          get_logger(),
          "Gyro bias estimated in lidar frame: [%.9f, %.9f, %.9f] rad/s from %lu static samples. IMU buffer cleared to avoid mixing pre-bias and post-bias samples.",
          gyro_bias_.x(),
          gyro_bias_.y(),
          gyro_bias_.z(),
          gyro_bias_sample_count_);
      } else {
        gyro_bias_.setZero();
        gyro_bias_estimated_ = false;
        gyro_bias_warning_printed_ = true;
        imu_buffer_.clear();
        RCLCPP_WARN(
          get_logger(),
          "Gyro bias could not be estimated: no static IMU samples in the first %.3f s. Continuing with zero bias and clearing early IMU buffer.",
          gyro_bias_estimation_duration_);
      }
    }
  } else if (!estimate_gyro_bias_ && !gyro_bias_warning_printed_) {
    gyro_bias_.setZero();
    gyro_bias_warning_printed_ = true;
    RCLCPP_INFO(get_logger(), "Gyro bias estimation disabled. Using zero gyro bias.");
  }

  ImuSample sample;
  sample.stamp = stamp;
  sample.angular_velocity_lidar = angular_velocity_lidar - gyro_bias_;
  sample.linear_acceleration_lidar = linear_acceleration_lidar;
  imu_buffer_.addSample(sample);
}

void DeskewNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (enable_time_check_) {
    recordCloudDelay(*msg);
    maybePrintTimeCheck();
  }

  PointCloudFieldHelper field_helper;
  std::string field_error;
  if (!field_helper.configure(*msg, point_time_field_, point_time_unit_, &field_error)) {
    handleDeskewFailure(*msg, "point cloud field configuration failed: " + field_error);
    return;
  }

  logFieldDetectionIfChanged(field_helper);

  if (!field_helper.hasXYZ()) {
    handleDeskewFailure(*msg, "point cloud is missing x/y/z fields");
    return;
  }

  const std::size_t point_count =
    static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
  if (point_count == 0) {
    auto output = *msg;
    output.header.frame_id = lidar_frame_;
    output.header.stamp =
      timeToMsg(rclcpp::Time(msg->header.stamp) + secondsToDuration(cloud_time_offset_));
    cloud_pub_->publish(output);
    RCLCPP_WARN(get_logger(), "Deskew skipped: input cloud has zero points.");
    return;
  }

  const std::size_t required_data_size =
    (static_cast<std::size_t>(msg->height) - 1U) * msg->row_step +
    static_cast<std::size_t>(msg->width) * msg->point_step;
  if (msg->data.size() < required_data_size) {
    handleDeskewFailure(*msg, "PointCloud2 data buffer is smaller than width/height/step metadata");
    return;
  }

  std::vector<double> point_offsets(point_count, 0.0);
  double min_point_offset = std::numeric_limits<double>::infinity();
  double max_point_offset = -std::numeric_limits<double>::infinity();

  if (!field_helper.hasPointTime()) {
    const std::string reason =
      "no per-point time field found. Tried configured field '" + point_time_field_ +
      "' and fallbacks offset_time/time/timestamp/t";
    if (drop_if_no_point_time_) {
      RCLCPP_WARN(get_logger(), "%s. Dropping cloud because drop_if_no_point_time=true.", reason.c_str());
      return;
    }
    RCLCPP_WARN(get_logger(), "%s. Degrading to zero point offsets.", reason.c_str());
    min_point_offset = 0.0;
    max_point_offset = 0.0;
  } else {
    for (std::uint32_t row = 0; row < msg->height; ++row) {
      for (std::uint32_t col = 0; col < msg->width; ++col) {
        const std::size_t index =
          static_cast<std::size_t>(row) * static_cast<std::size_t>(msg->width) +
          static_cast<std::size_t>(col);
        const std::uint8_t * point =
          msg->data.data() + static_cast<std::size_t>(row) * msg->row_step +
          static_cast<std::size_t>(col) * msg->point_step;

        double offset_seconds = 0.0;
        if (!field_helper.readPointTime(point, offset_seconds)) {
          handleDeskewFailure(*msg, "failed to read per-point time field");
          return;
        }
        point_offsets[index] = offset_seconds;
        min_point_offset = std::min(min_point_offset, offset_seconds);
        max_point_offset = std::max(max_point_offset, offset_seconds);
      }
    }
  }

  if (!std::isfinite(min_point_offset) || !std::isfinite(max_point_offset)) {
    handleDeskewFailure(*msg, "invalid point time range");
    return;
  }

  const double scan_duration = std::max(0.0, max_point_offset - min_point_offset);
  const rclcpp::Time adjusted_cloud_stamp =
    rclcpp::Time(msg->header.stamp) + secondsToDuration(cloud_time_offset_);

  // cloud.header.stamp 可能表示帧首或帧尾；scan_duration 由每点时间范围实时计算，不硬编码。
  rclcpp::Time actual_scan_start = adjusted_cloud_stamp;
  rclcpp::Time actual_scan_end = adjusted_cloud_stamp;
  if (cloud_stamp_type_ == "scan_end") {
    actual_scan_end = adjusted_cloud_stamp;
    actual_scan_start = actual_scan_end - secondsToDuration(scan_duration);
  } else {
    actual_scan_start = adjusted_cloud_stamp;
    actual_scan_end = actual_scan_start + secondsToDuration(scan_duration);
  }

  rclcpp::Time t_ref = actual_scan_end;
  if (deskew_target_time_ == "scan_start") {
    t_ref = actual_scan_start;
  } else if (deskew_target_time_ == "scan_mid") {
    t_ref = actual_scan_start + secondsToDuration(scan_duration * 0.5);
  }

  if (enable_debug_log_) {
    logCloudTiming(
      *msg,
      actual_scan_start,
      actual_scan_end,
      scan_duration,
      min_point_offset,
      max_point_offset);
  }

  // livox_ros_driver2 的 PointCloud2 timestamp 可能是绝对点时间；减去本帧最小值后统一成帧内相对时间。
  for (double & offset : point_offsets) {
    offset -= min_point_offset;
  }

  Eigen::Isometry3d t_base_lidar = Eigen::Isometry3d::Identity();
  std::string tf_reason;
  if (!lookupBaseToLidar(t_base_lidar, tf_reason)) {
    handleDeskewFailure(*msg, "failed to lookup base_link -> mid360_link extrinsic: " + tf_reason);
    return;
  }

  DeskewTiming timing;
  timing.actual_scan_start = actual_scan_start;
  timing.actual_scan_end = actual_scan_end;
  timing.t_ref = t_ref;
  timing.scan_duration = scan_duration;

  sensor_msgs::msg::PointCloud2 output;
  std::string primary_reason;
  FailureKind primary_failure_kind = FailureKind::Generic;
  bool success = deskewWithMethod(
    deskew_method_,
    *msg,
    field_helper,
    point_offsets,
    timing,
    t_base_lidar,
    output,
    primary_reason,
    primary_failure_kind);
  std::string used_method = deskew_method_;

  if (!success && fallback_method_ != "none" && fallback_method_ != deskew_method_) {
    RCLCPP_WARN(
      get_logger(),
      "Primary deskew_method '%s' failed: %s. Trying fallback_method '%s'.",
      deskew_method_.c_str(),
      primary_reason.c_str(),
      fallback_method_.c_str());

    std::string fallback_reason;
    FailureKind fallback_failure_kind = FailureKind::Generic;
    success = deskewWithMethod(
      fallback_method_,
      *msg,
      field_helper,
      point_offsets,
      timing,
      t_base_lidar,
      output,
      fallback_reason,
      fallback_failure_kind);
    if (success) {
      used_method = fallback_method_;
      RCLCPP_WARN(
        get_logger(),
        "Deskew succeeded with fallback_method '%s'.",
        fallback_method_.c_str());
    } else {
      handleDeskewFailure(
        *msg,
        "primary method '" + deskew_method_ + "' failed: " + primary_reason +
          "; fallback method '" + fallback_method_ + "' failed: " + fallback_reason,
        fallback_failure_kind);
      return;
    }
  } else if (!success) {
    handleDeskewFailure(
      *msg,
      "method '" + deskew_method_ + "' failed: " + primary_reason,
      primary_failure_kind);
    return;
  }

  cloud_pub_->publish(output);

  if (enable_debug_log_) {
    RCLCPP_INFO(
      get_logger(),
      "Deskew success: method=%s stamp=%s frame=%s",
      used_method.c_str(),
      stampToString(t_ref).c_str(),
      lidar_frame_.c_str());
    logPoseBufferStatus("deskew success");
    logImuBufferStatus("deskew success");
  }
}

void DeskewNode::customMsgCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
  std_msgs::msg::Header input_header = msg->header;
  const std::string source_frame = input_header.frame_id.empty() ?
    (custom_msg_frame_id_.empty() ? lidar_frame_ : custom_msg_frame_id_) :
    input_header.frame_id;
  if (input_header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "CustomMsg header.frame_id is empty. Using custom_msg_frame_id='%s' for diagnostics; output frame_id remains lidar_frame='%s'.",
      source_frame.c_str(),
      lidar_frame_.c_str());
  }
  input_header.frame_id = source_frame;
  if (source_frame != lidar_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "CustomMsg frame_id='%s' differs from lidar_frame='%s'. Dropping cloud because source->lidar point transform is not implemented.",
      source_frame.c_str(),
      lidar_frame_.c_str());
    return;
  }

  rclcpp::Time adjusted_cloud_stamp(msg->header.stamp);
  if (custom_msg_timebase_type_ == "custom_timebase") {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      10000,
      "custom_msg_timebase_type='custom_timebase' is configured, but this version does not use Livox timebase to query Odin1 odom. Falling back to msg.header.stamp. Use custom_timebase only after Livox timebase is synchronized with ROS time/Odin1 odom.");
  }
  adjusted_cloud_stamp = adjusted_cloud_stamp + secondsToDuration(cloud_time_offset_);

  if (enable_time_check_) {
    recordCloudDelay(adjusted_cloud_stamp);
    maybePrintTimeCheck();
  }

  const std::size_t point_count = msg->points.size();
  if (msg->point_num != point_count) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "CustomMsg point_num=%u differs from points.size()=%zu. Using points.size().",
      msg->point_num,
      point_count);
  }

  std::vector<DeskewPoint> points;
  points.reserve(point_count);
  if (point_count == 0) {
    auto output = buildCustomOutputCloud(points, adjusted_cloud_stamp, lidar_frame_);
    cloud_pub_->publish(output);
    RCLCPP_WARN(get_logger(), "Deskew skipped: CustomMsg has zero points.");
    return;
  }

  double offset_scale = 0.0;
  if (!customMsgOffsetUnitScale(custom_msg_offset_unit_, offset_scale)) {
    handleCustomDeskewFailure(
      input_header,
      points,
      adjusted_cloud_stamp,
      "unsupported custom_msg_offset_unit: " + custom_msg_offset_unit_);
    return;
  }

  std::uint32_t min_offset_raw = std::numeric_limits<std::uint32_t>::max();
  std::uint32_t max_offset_raw = 0U;
  for (const auto & livox_point : msg->points) {
    DeskewPoint point;
    point.x = livox_point.x;
    point.y = livox_point.y;
    point.z = livox_point.z;
    point.intensity = static_cast<float>(livox_point.reflectivity);
    point.offset_time_raw = livox_point.offset_time;
    point.offset_time_sec = static_cast<double>(livox_point.offset_time) * offset_scale;
    point.tag = livoxTagOrZero(livox_point);
    point.line = livoxLineOrZero(livox_point);
    point.valid =
      std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    points.push_back(point);

    min_offset_raw = std::min(min_offset_raw, livox_point.offset_time);
    max_offset_raw = std::max(max_offset_raw, livox_point.offset_time);
  }

  const double scan_duration =
    static_cast<double>(max_offset_raw - min_offset_raw) * offset_scale;
  if (scan_duration <= 0.0) {
    handleCustomDeskewFailure(
      input_header,
      points,
      adjusted_cloud_stamp,
      "CustomMsg scan_duration <= 0 after offset_time normalization");
    return;
  }
  if (scan_duration > 0.2) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "CustomMsg scan_duration=%.6f s is unusually large for MID360. Check custom_msg_offset_unit and cloud_stamp_type.",
      scan_duration);
  }

  // CustomMsg offset_time is a per-point relative offset, not an absolute ROS timestamp.
  for (auto & point : points) {
    point.offset_time_sec =
      static_cast<double>(point.offset_time_raw - min_offset_raw) * offset_scale;
  }

  rclcpp::Time actual_scan_start = adjusted_cloud_stamp;
  rclcpp::Time actual_scan_end = adjusted_cloud_stamp;
  if (cloud_stamp_type_ == "scan_end") {
    actual_scan_end = adjusted_cloud_stamp;
    actual_scan_start = actual_scan_end - secondsToDuration(scan_duration);
  } else {
    actual_scan_start = adjusted_cloud_stamp;
    actual_scan_end = actual_scan_start + secondsToDuration(scan_duration);
  }

  rclcpp::Time t_ref = actual_scan_end;
  if (deskew_target_time_ == "scan_start") {
    t_ref = actual_scan_start;
  } else if (deskew_target_time_ == "scan_mid") {
    t_ref = actual_scan_start + secondsToDuration(scan_duration * 0.5);
  }

  if (enable_debug_log_) {
    logCustomMsgTiming(
      *msg,
      adjusted_cloud_stamp,
      actual_scan_start,
      actual_scan_end,
      scan_duration,
      min_offset_raw,
      max_offset_raw);
  }

  Eigen::Isometry3d t_base_lidar = Eigen::Isometry3d::Identity();
  std::string tf_reason;
  if (!lookupBaseToLidar(t_base_lidar, tf_reason)) {
    handleCustomDeskewFailure(
      input_header,
      points,
      t_ref,
      "failed to lookup base_link -> mid360_link extrinsic: " + tf_reason);
    return;
  }

  DeskewTiming timing;
  timing.actual_scan_start = actual_scan_start;
  timing.actual_scan_end = actual_scan_end;
  timing.t_ref = t_ref;
  timing.scan_duration = scan_duration;

  sensor_msgs::msg::PointCloud2 output;
  std::string primary_reason;
  FailureKind primary_failure_kind = FailureKind::Generic;
  bool success = deskewCustomPointsWithMethod(
    deskew_method_,
    points,
    timing,
    t_base_lidar,
    output,
    primary_reason,
    primary_failure_kind);
  std::string used_method = deskew_method_;

  if (!success && fallback_method_ != "none" && fallback_method_ != deskew_method_) {
    RCLCPP_WARN(
      get_logger(),
      "Primary deskew_method '%s' failed for CustomMsg: %s. Trying fallback_method '%s'.",
      deskew_method_.c_str(),
      primary_reason.c_str(),
      fallback_method_.c_str());

    std::string fallback_reason;
    FailureKind fallback_failure_kind = FailureKind::Generic;
    success = deskewCustomPointsWithMethod(
      fallback_method_,
      points,
      timing,
      t_base_lidar,
      output,
      fallback_reason,
      fallback_failure_kind);
    if (success) {
      used_method = fallback_method_;
      RCLCPP_WARN(
        get_logger(),
        "CustomMsg deskew succeeded with fallback_method '%s'.",
        fallback_method_.c_str());
    } else {
      handleCustomDeskewFailure(
        input_header,
        points,
        t_ref,
        "primary method '" + deskew_method_ + "' failed: " + primary_reason +
          "; fallback method '" + fallback_method_ + "' failed: " + fallback_reason,
        fallback_failure_kind);
      return;
    }
  } else if (!success) {
    handleCustomDeskewFailure(
      input_header,
      points,
      t_ref,
      "method '" + deskew_method_ + "' failed: " + primary_reason,
      primary_failure_kind);
    return;
  }

  cloud_pub_->publish(output);

  if (enable_debug_log_) {
    RCLCPP_INFO(
      get_logger(),
      "CustomMsg deskew success: method=%s stamp=%s frame=%s input_points=%zu output_points=%u",
      used_method.c_str(),
      stampToString(t_ref).c_str(),
      lidar_frame_.c_str(),
      point_count,
      output.width);
    logPoseBufferStatus("custom_msg deskew success");
    logImuBufferStatus("custom_msg deskew success");
  }
}

bool DeskewNode::deskewWithMethod(
  const std::string & method,
  const sensor_msgs::msg::PointCloud2 & cloud,
  const PointCloudFieldHelper & field_helper,
  const std::vector<double> & point_offsets,
  const DeskewTiming & timing,
  const Eigen::Isometry3d & t_base_lidar,
  sensor_msgs::msg::PointCloud2 & output,
  std::string & reason,
  FailureKind & failure_kind)
{
  const std::size_t point_count =
    static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
  if (point_offsets.size() != point_count) {
    reason = "point offset count does not match PointCloud2 point count";
    failure_kind = FailureKind::Generic;
    return false;
  }

  std::vector<DeskewPoint> points;
  points.reserve(point_count);
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      const std::size_t index =
        static_cast<std::size_t>(row) * static_cast<std::size_t>(cloud.width) +
        static_cast<std::size_t>(col);
      const std::uint8_t * input_point =
        cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step +
        static_cast<std::size_t>(col) * cloud.point_step;

      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      if (!field_helper.readXYZ(input_point, x, y, z)) {
        reason = "failed to read x/y/z fields";
        failure_kind = FailureKind::Generic;
        return false;
      }

      DeskewPoint point;
      point.x = static_cast<float>(x);
      point.y = static_cast<float>(y);
      point.z = static_cast<float>(z);
      point.offset_time_sec = point_offsets[index];
      point.valid = std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
      points.push_back(point);
    }
  }

  std::vector<DeskewPoint> corrected_points;
  DeskewStats stats;
  if (!processDeskewPoints(
      method,
      points,
      timing,
      t_base_lidar,
      corrected_points,
      stats,
      reason,
      failure_kind))
  {
    return false;
  }

  output = cloud;
  output.header.frame_id = lidar_frame_;
  output.header.stamp = timeToMsg(timing.t_ref);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      const std::size_t index =
        static_cast<std::size_t>(row) * static_cast<std::size_t>(cloud.width) +
        static_cast<std::size_t>(col);
      std::uint8_t * output_point =
        output.data.data() + static_cast<std::size_t>(row) * output.row_step +
        static_cast<std::size_t>(col) * output.point_step;

      const auto & corrected = corrected_points[index];
      if (!corrected.valid) {
        output.is_dense = false;
        if (corrected.output_nan) {
          if (!field_helper.writeXYZ(output_point, nan, nan, nan)) {
            reason = "x/y/z fields must be FLOAT32 or FLOAT64 for writing";
            failure_kind = FailureKind::Generic;
            return false;
          }
        }
        continue;
      }

      if (!field_helper.writeXYZ(
          output_point,
          corrected.x,
          corrected.y,
          corrected.z))
      {
        reason = "x/y/z fields must be FLOAT32 or FLOAT64 for writing";
        failure_kind = FailureKind::Generic;
        return false;
      }
    }
  }

  if (enable_debug_log_) {
    RCLCPP_INFO(
      get_logger(),
      "Deskew method '%s' completed: valid=%zu nan_or_inf=%zu range_filtered=%zu",
      method.c_str(),
      stats.valid_points,
      stats.nan_points,
      stats.range_filtered_points);
  }

  return true;
}

bool DeskewNode::deskewCustomPointsWithMethod(
  const std::string & method,
  const std::vector<DeskewPoint> & points,
  const DeskewTiming & timing,
  const Eigen::Isometry3d & t_base_lidar,
  sensor_msgs::msg::PointCloud2 & output,
  std::string & reason,
  FailureKind & failure_kind)
{
  std::vector<DeskewPoint> corrected_points;
  DeskewStats stats;
  if (!processDeskewPoints(
      method,
      points,
      timing,
      t_base_lidar,
      corrected_points,
      stats,
      reason,
      failure_kind))
  {
    return false;
  }

  output = buildCustomOutputCloud(corrected_points, timing.t_ref, lidar_frame_);

  if (enable_debug_log_) {
    RCLCPP_INFO(
      get_logger(),
      "CustomMsg deskew method '%s' completed: valid=%zu nan_or_inf=%zu range_filtered=%zu filter_mode=%s",
      method.c_str(),
      stats.valid_points,
      stats.nan_points,
      stats.range_filtered_points,
      filter_mode_.c_str());
  }

  return true;
}

bool DeskewNode::processDeskewPoints(
  const std::string & method,
  const std::vector<DeskewPoint> & points,
  const DeskewTiming & timing,
  const Eigen::Isometry3d & t_base_lidar,
  std::vector<DeskewPoint> & corrected_points,
  DeskewStats & stats,
  std::string & reason,
  FailureKind & failure_kind)
{
  failure_kind = FailureKind::Generic;

  if (method != "odom" && method != "linear" && method != "imu_only" && method != "odom_imu") {
    reason = "unsupported deskew method: " + method;
    return false;
  }

  const bool needs_odom = method == "odom" || method == "linear" || method == "odom_imu";
  const bool needs_imu = method == "imu_only" || method == "odom_imu";
  const auto poses = needs_odom ? pose_buffer_.snapshot() : std::vector<Pose>{};
  const auto imu_samples = needs_imu ? imu_buffer_.snapshot() : std::vector<ImuSample>{};

  auto lookup_odom_pose =
    [&](const rclcpp::Time & stamp, Pose & pose, std::string & lookup_reason) -> bool {
      const rclcpp::Time query_time = stamp + secondsToDuration(odomTimeOffsetRuntime());
      if (!PoseBuffer::lookupInPoses(
          poses,
          query_time,
          pose_buffer_.maxAllowedTimeGap(),
          pose_buffer_.allowExtrapolation(),
          pose,
          &lookup_reason))
      {
        ++interpolation_failed_count_;
        failure_kind = FailureKind::NoPose;
        return false;
      }
      return true;
    };

  auto integrate_imu =
    [&](const rclcpp::Time & from,
        const rclcpp::Time & to,
        Eigen::Quaterniond & q_from_to,
        std::string & imu_reason) -> bool {
      const rclcpp::Time from_query = from + secondsToDuration(imuTimeOffsetRuntime());
      const rclcpp::Time to_query = to + secondsToDuration(imuTimeOffsetRuntime());
      if (!ImuBuffer::integrateRotation(
          imu_samples,
          from_query,
          to_query,
          imu_buffer_.maxAllowedTimeGap(),
          q_from_to,
          &imu_reason))
      {
        ++imu_integration_failed_count_;
        failure_kind = FailureKind::NoImu;
        return false;
      }
      if (imu_rotation_sign_ < 0.0) {
        q_from_to = q_from_to.inverse();
      }
      q_from_to.normalize();
      return true;
    };

  Pose ref_pose;
  Pose linear_start_pose;
  Pose linear_end_pose;
  Eigen::Isometry3d t_source_lidar_ref = Eigen::Isometry3d::Identity();
  std::string lookup_reason;

  if (method == "odom") {
    if (!lookup_odom_pose(timing.t_ref, ref_pose, lookup_reason)) {
      logPoseBufferStatus("reference pose lookup failed");
      reason = "failed to interpolate odom pose at t_ref: " + lookup_reason;
      return false;
    }
    t_source_lidar_ref = poseToIsometry(ref_pose) * t_base_lidar;
  } else if (method == "linear") {
    if (!lookup_odom_pose(timing.actual_scan_start, linear_start_pose, lookup_reason)) {
      logPoseBufferStatus("linear fallback scan_start pose lookup failed");
      reason = "failed to interpolate odom pose at scan_start for linear fallback: " +
        lookup_reason;
      return false;
    }

    if (!lookup_odom_pose(timing.actual_scan_end, linear_end_pose, lookup_reason)) {
      logPoseBufferStatus("linear fallback scan_end pose lookup failed");
      reason = "failed to interpolate odom pose at scan_end for linear fallback: " +
        lookup_reason;
      return false;
    }

    const double ref_alpha = timing.scan_duration > 0.0 ?
      clamp01((timing.t_ref - timing.actual_scan_start).seconds() / timing.scan_duration) :
      0.0;
    ref_pose = interpolatePose(linear_start_pose, linear_end_pose, ref_alpha, timing.t_ref);
    t_source_lidar_ref = poseToIsometry(ref_pose) * t_base_lidar;
  } else if (method == "imu_only") {
    Eigen::Quaterniond q_scan_check = Eigen::Quaterniond::Identity();
    std::string imu_reason;
    if (!integrate_imu(timing.actual_scan_start, timing.actual_scan_end, q_scan_check, imu_reason)) {
      logImuBufferStatus("imu_only scan coverage check failed");
      reason = "failed to integrate IMU rotation over full scan: " + imu_reason;
      return false;
    }
    logImuRotationDebug("imu_only full_scan", timing.actual_scan_start, timing.actual_scan_end, q_scan_check);

    Eigen::Quaterniond q_start_ref = Eigen::Quaterniond::Identity();
    if (!integrate_imu(timing.actual_scan_start, timing.t_ref, q_start_ref, imu_reason)) {
      logImuBufferStatus("imu_only reference rotation integration failed");
      reason = "failed to integrate IMU rotation at t_ref: " + imu_reason;
      return false;
    }
    (void)q_start_ref;
    t_source_lidar_ref = Eigen::Isometry3d::Identity();
  } else if (method == "odom_imu") {
    if (!lookup_odom_pose(timing.t_ref, ref_pose, lookup_reason)) {
      logPoseBufferStatus("odom_imu reference pose lookup failed");
      reason = "failed to interpolate odom pose at t_ref for odom_imu: " + lookup_reason;
      return false;
    }

    Eigen::Quaterniond q_scan_check = Eigen::Quaterniond::Identity();
    std::string imu_reason;
    if (!integrate_imu(timing.actual_scan_start, timing.actual_scan_end, q_scan_check, imu_reason)) {
      logImuBufferStatus("odom_imu scan coverage check failed");
      reason = "failed to integrate IMU rotation over full scan for odom_imu: " + imu_reason;
      return false;
    }
    logImuRotationDebug("odom_imu full_scan", timing.actual_scan_start, timing.actual_scan_end, q_scan_check);

    t_source_lidar_ref = poseToIsometry(ref_pose) * t_base_lidar;
  }

  const Eigen::Isometry3d t_lidar_ref_source = t_source_lidar_ref.inverse();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  bool logged_point_imu_rotation = false;
  corrected_points.reserve(points.size());

  for (const auto & point : points) {
    DeskewPoint corrected = point;

    const bool finite_point =
      point.valid && std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    if (!finite_point) {
      ++stats.nan_points;
      corrected.valid = false;
      corrected.output_nan = remove_nan_;
      if (corrected.output_nan) {
        corrected.x = nan;
        corrected.y = nan;
        corrected.z = nan;
      }
      corrected_points.push_back(corrected);
      continue;
    }

    const double x = static_cast<double>(point.x);
    const double y = static_cast<double>(point.y);
    const double z = static_cast<double>(point.z);
    const double range = std::sqrt(x * x + y * y + z * z);
    if (range < min_range_ || range > max_range_) {
      ++stats.range_filtered_points;
      corrected.x = nan;
      corrected.y = nan;
      corrected.z = nan;
      corrected.valid = false;
      corrected.output_nan = true;
      corrected_points.push_back(corrected);
      continue;
    }

    const rclcpp::Time point_time =
      timing.actual_scan_start + secondsToDuration(point.offset_time_sec);
    Eigen::Isometry3d t_source_lidar_i = Eigen::Isometry3d::Identity();

    if (method == "odom") {
      Pose point_pose;
      if (!lookup_odom_pose(point_time, point_pose, lookup_reason)) {
        logPoseBufferStatus("point pose lookup failed");
        reason = "failed to interpolate odom pose at point time: " + lookup_reason;
        return false;
      }
      t_source_lidar_i = poseToIsometry(point_pose) * t_base_lidar;
    } else if (method == "linear") {
      const double alpha = timing.scan_duration > 0.0 ?
        clamp01((point_time - timing.actual_scan_start).seconds() / timing.scan_duration) :
        0.0;
      const Pose point_pose =
        interpolatePose(linear_start_pose, linear_end_pose, alpha, point_time);
      t_source_lidar_i = poseToIsometry(point_pose) * t_base_lidar;
    } else if (method == "imu_only") {
      Eigen::Quaterniond q_ref_point = Eigen::Quaterniond::Identity();
      std::string imu_reason;
      if (!integrate_imu(timing.t_ref, point_time, q_ref_point, imu_reason)) {
        logImuBufferStatus("imu_only point rotation integration failed");
        reason = "failed to integrate IMU rotation at point time: " + imu_reason;
        return false;
      }
      if (!logged_point_imu_rotation) {
        logImuRotationDebug("imu_only sample_point", timing.t_ref, point_time, q_ref_point);
        logged_point_imu_rotation = true;
      }
      t_source_lidar_i = rotationOnlyPose(q_ref_point);
    } else if (method == "odom_imu") {
      Pose point_pose;
      if (!lookup_odom_pose(point_time, point_pose, lookup_reason)) {
        logPoseBufferStatus("odom_imu point pose lookup failed");
        reason = "failed to interpolate odom position at point time for odom_imu: " +
          lookup_reason;
        return false;
      }

      Eigen::Quaterniond q_ref_point = Eigen::Quaterniond::Identity();
      std::string imu_reason;
      if (!integrate_imu(timing.t_ref, point_time, q_ref_point, imu_reason)) {
        logImuBufferStatus("odom_imu point rotation integration failed");
        reason = "failed to integrate IMU relative rotation at point time: " + imu_reason;
        return false;
      }
      if (!logged_point_imu_rotation) {
        logImuRotationDebug("odom_imu sample_point", timing.t_ref, point_time, q_ref_point);
        logged_point_imu_rotation = true;
      }

      const Eigen::Isometry3d t_odom_lidar_ref = poseToIsometry(ref_pose) * t_base_lidar;
      const Eigen::Isometry3d t_odom_lidar_i = poseToIsometry(point_pose) * t_base_lidar;
      const Eigen::Isometry3d t_rel_odom = t_odom_lidar_ref.inverse() * t_odom_lidar_i;

      t_source_lidar_i = Eigen::Isometry3d::Identity();
      t_source_lidar_i.translation() = t_rel_odom.translation();
      if (rotation_source_ == "odom") {
        t_source_lidar_i.linear() = t_rel_odom.linear();
      } else if (rotation_source_ == "fused") {
        Eigen::Quaterniond q_odom(t_rel_odom.linear());
        q_odom.normalize();
        t_source_lidar_i.linear() = q_odom.slerp(0.5, q_ref_point.normalized()).toRotationMatrix();
      } else {
        t_source_lidar_i.linear() = q_ref_point.normalized().toRotationMatrix();
      }
    }

    const Eigen::Vector3d raw_point(x, y, z);
    Eigen::Vector3d corrected_vector = Eigen::Vector3d::Zero();
    if (method == "imu_only" || method == "odom_imu") {
      corrected_vector = t_source_lidar_i * raw_point;
    } else {
      corrected_vector = t_lidar_ref_source * t_source_lidar_i * raw_point;
    }

    if (!std::isfinite(corrected_vector.x()) ||
        !std::isfinite(corrected_vector.y()) ||
        !std::isfinite(corrected_vector.z()))
    {
      ++stats.nan_points;
      corrected.x = nan;
      corrected.y = nan;
      corrected.z = nan;
      corrected.valid = false;
      corrected.output_nan = true;
      corrected_points.push_back(corrected);
      continue;
    }

    corrected.x = static_cast<float>(corrected_vector.x());
    corrected.y = static_cast<float>(corrected_vector.y());
    corrected.z = static_cast<float>(corrected_vector.z());
    corrected.valid = true;
    corrected.output_nan = false;
    corrected_points.push_back(corrected);
    ++stats.valid_points;
  }

  return true;
}

sensor_msgs::msg::PointCloud2 DeskewNode::buildCustomOutputCloud(
  const std::vector<DeskewPoint> & points,
  const rclcpp::Time & stamp,
  const std::string & frame_id) const
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = timeToMsg(stamp);
  cloud.height = 1U;
  cloud.is_bigendian = false;
  cloud.is_dense = false;

  const bool compact = filter_mode_ == "compact";
  const std::uint32_t point_step = 24U;
  cloud.fields.resize(7U);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0U;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1U;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4U;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1U;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8U;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1U;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].offset = 12U;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[3].count = 1U;
  cloud.fields[4].name = "offset_time";
  cloud.fields[4].offset = 16U;
  cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT32;
  cloud.fields[4].count = 1U;
  cloud.fields[5].name = "tag";
  cloud.fields[5].offset = 20U;
  cloud.fields[5].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.fields[5].count = 1U;
  cloud.fields[6].name = "line";
  cloud.fields[6].offset = 21U;
  cloud.fields[6].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.fields[6].count = 1U;
  cloud.point_step = point_step;

  std::uint32_t output_count = 0U;
  if (compact) {
    output_count = static_cast<std::uint32_t>(
      std::count_if(points.begin(), points.end(), [](const DeskewPoint & point) {
        return point.valid;
      }));
  } else {
    output_count = static_cast<std::uint32_t>(points.size());
  }

  cloud.width = output_count;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step) * cloud.height, 0U);

  std::size_t output_index = 0U;
  for (const auto & point : points) {
    if (compact && !point.valid) {
      continue;
    }

    const std::size_t base = output_index * cloud.point_step;
    writePointField(cloud.data, base + 0U, point.x);
    writePointField(cloud.data, base + 4U, point.y);
    writePointField(cloud.data, base + 8U, point.z);
    writePointField(cloud.data, base + 12U, point.intensity);
    writePointField(cloud.data, base + 16U, point.offset_time_raw);
    writePointField(cloud.data, base + 20U, point.tag);
    writePointField(cloud.data, base + 21U, point.line);
    ++output_index;
  }

  return cloud;
}

bool DeskewNode::lookupBaseToLidar(Eigen::Isometry3d & t_base_lidar, std::string & reason)
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      base_frame_,
      lidar_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.05));
    t_base_lidar = transformToEigen(transform);
    return true;
  } catch (const tf2::TransformException & ex) {
    reason = ex.what();
    return false;
  }
}

void DeskewNode::handleDeskewFailure(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & reason,
  FailureKind kind)
{
  RCLCPP_WARN(get_logger(), "Deskew failed/dropped: %s", reason.c_str());

  if (kind == FailureKind::NoPose && drop_if_no_pose_) {
    return;
  }

  if (kind == FailureKind::NoImu && drop_if_no_imu_) {
    return;
  }

  if (publish_raw_when_failed_) {
    sensor_msgs::msg::PointCloud2 raw = cloud;
    raw.header.frame_id = lidar_frame_;
    RCLCPP_WARN(get_logger(), "Publishing raw MID360 cloud on output topic due to failure.");
    cloud_pub_->publish(raw);
    return;
  }

  RCLCPP_WARN(get_logger(), "No safe fallback enabled; dropping failed cloud.");
}

void DeskewNode::handleCustomDeskewFailure(
  const std_msgs::msg::Header & header,
  const std::vector<DeskewPoint> & points,
  const rclcpp::Time & output_stamp,
  const std::string & reason,
  FailureKind kind)
{
  (void)header;
  RCLCPP_WARN(get_logger(), "CustomMsg deskew failed/dropped: %s", reason.c_str());

  if (kind == FailureKind::NoPose && drop_if_no_pose_) {
    return;
  }

  if (kind == FailureKind::NoImu && drop_if_no_imu_) {
    return;
  }

  if (publish_raw_when_failed_) {
    auto raw = buildCustomOutputCloud(points, output_stamp, lidar_frame_);
    RCLCPP_WARN(get_logger(), "Publishing raw CustomMsg-converted cloud on output topic due to failure.");
    cloud_pub_->publish(raw);
    return;
  }

  RCLCPP_WARN(get_logger(), "No safe fallback enabled; dropping failed CustomMsg cloud.");
}

void DeskewNode::recordCloudDelay(const sensor_msgs::msg::PointCloud2 & cloud)
{
  recordCloudDelay(rclcpp::Time(cloud.header.stamp));
}

void DeskewNode::recordCloudDelay(const rclcpp::Time & stamp)
{
  const rclcpp::Time now = this->now();
  const double delay = (now - stamp).seconds();
  if (delay < 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Negative delay detected. Please check time base or use_sim_time. cloud_delay=%.6f s",
      delay);
  }
  cloud_delay_stats_.addSample(now, delay, time_check_window_);
}

void DeskewNode::recordOdomDelay(const nav_msgs::msg::Odometry & odom)
{
  const rclcpp::Time now = this->now();
  const rclcpp::Time stamp(odom.header.stamp);
  const double delay = (now - stamp).seconds();
  if (delay < 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Negative delay detected. Please check time base or use_sim_time. odom_delay=%.6f s",
      delay);
  }
  odom_delay_stats_.addSample(now, delay, time_check_window_);
}

void DeskewNode::recordImuDelay(const sensor_msgs::msg::Imu & imu)
{
  const rclcpp::Time now = this->now();
  const rclcpp::Time stamp(imu.header.stamp);
  const double delay = (now - stamp).seconds();
  if (delay < 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Negative delay detected. Please check time base or use_sim_time. imu_delay=%.6f s",
      delay);
  }
  imu_delay_stats_.addSample(now, delay, time_check_window_);
}

void DeskewNode::maybePrintTimeCheck()
{
  if (!enable_time_check_) {
    return;
  }

  const rclcpp::Time now = this->now();
  cloud_delay_stats_.prune(now, time_check_window_);
  odom_delay_stats_.prune(now, time_check_window_);
  imu_delay_stats_.prune(now, time_check_window_);

  const auto cloud_stats = cloud_delay_stats_.calculate();
  const auto odom_stats = odom_delay_stats_.calculate();
  const auto imu_stats = imu_delay_stats_.calculate();
  // odom-only 调试时可能没有 IMU；cloud/odom 的时间建议不应被 IMU 样本不足阻塞。
  const bool has_imu_time_stats =
    imu_stats.sample_count >= static_cast<std::size_t>(time_check_min_samples_);

  if (cloud_stats.sample_count < static_cast<std::size_t>(time_check_min_samples_) ||
      odom_stats.sample_count < static_cast<std::size_t>(time_check_min_samples_))
  {
    return;
  }

  if (last_time_check_print_.nanoseconds() != 0 &&
      (now - last_time_check_print_).seconds() < time_check_window_)
  {
    return;
  }
  last_time_check_print_ = now;

  const double recommended_odom_time_offset = odom_stats.mean - cloud_stats.mean;
  const double recommended_imu_time_offset =
    has_imu_time_stats ? imu_stats.mean - cloud_stats.mean : 0.0;

  if (print_time_check_result_) {
    RCLCPP_INFO(
      get_logger(),
      "time_check: avg_cloud_delay=%.6f std_cloud_delay=%.6f min_cloud_delay=%.6f max_cloud_delay=%.6f samples=%zu",
      cloud_stats.mean,
      cloud_stats.stddev,
      cloud_stats.min,
      cloud_stats.max,
      cloud_stats.sample_count);
    RCLCPP_INFO(
      get_logger(),
      "time_check: avg_odom_delay=%.6f std_odom_delay=%.6f min_odom_delay=%.6f max_odom_delay=%.6f samples=%zu",
      odom_stats.mean,
      odom_stats.stddev,
      odom_stats.min,
      odom_stats.max,
      odom_stats.sample_count);
    if (has_imu_time_stats) {
      RCLCPP_INFO(
        get_logger(),
        "time_check: avg_imu_delay=%.6f std_imu_delay=%.6f min_imu_delay=%.6f max_imu_delay=%.6f samples=%zu",
        imu_stats.mean,
        imu_stats.stddev,
        imu_stats.min,
        imu_stats.max,
        imu_stats.sample_count);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "time_check: IMU samples are insufficient for imu_time_offset recommendation. imu_samples=%zu required=%d",
        imu_stats.sample_count,
        time_check_min_samples_);
    }
    RCLCPP_INFO(
      get_logger(),
      "Recommended value for odom_time_offset: %.6f seconds",
      recommended_odom_time_offset);
    if (has_imu_time_stats) {
      RCLCPP_INFO(
        get_logger(),
        "Recommended value for imu_time_offset: %.6f seconds",
        recommended_imu_time_offset);
    }
    if (!apply_auto_time_offset_) {
      RCLCPP_INFO(
        get_logger(),
        "You may set odom_time_offset: %.6f in YAML",
        recommended_odom_time_offset);
      if (has_imu_time_stats) {
        RCLCPP_INFO(
          get_logger(),
          "You may set imu_time_offset: %.6f in YAML",
          recommended_imu_time_offset);
      }
    }
  }

  if (std::abs(recommended_odom_time_offset) > auto_time_offset_max_abs_) {
    RCLCPP_WARN(
      get_logger(),
      "Estimated odom time offset is too large; time base may be inconsistent. estimated=%.6f max_abs=%.6f",
      recommended_odom_time_offset,
      auto_time_offset_max_abs_);
    if (apply_auto_time_offset_) {
      odom_time_offset_auto_ = 0.0;
    }
  } else if (apply_auto_time_offset_) {
    odom_time_offset_auto_ = recommended_odom_time_offset - odom_time_offset_config_;
    RCLCPP_INFO(
      get_logger(),
      "Applied runtime odom_time_offset: config=%.6f auto=%.6f runtime=%.6f",
      odom_time_offset_config_,
      odom_time_offset_auto_,
      odomTimeOffsetRuntime());
  }

  if (has_imu_time_stats && std::abs(recommended_imu_time_offset) > auto_time_offset_max_abs_) {
    RCLCPP_WARN(
      get_logger(),
      "Estimated imu time offset is too large; time base may be inconsistent. estimated=%.6f max_abs=%.6f",
      recommended_imu_time_offset,
      auto_time_offset_max_abs_);
    if (apply_auto_time_offset_) {
      imu_time_offset_auto_ = 0.0;
    }
  } else if (has_imu_time_stats && apply_auto_time_offset_) {
    imu_time_offset_auto_ = recommended_imu_time_offset - imu_time_offset_config_;
    RCLCPP_INFO(
      get_logger(),
      "Applied runtime imu_time_offset: config=%.6f auto=%.6f runtime=%.6f",
      imu_time_offset_config_,
      imu_time_offset_auto_,
      imuTimeOffsetRuntime());
  }

  if (cloud_stats.stddev > delay_jitter_warning_threshold_ ||
      odom_stats.stddev > delay_jitter_warning_threshold_ ||
      (has_imu_time_stats && imu_stats.stddev > delay_jitter_warning_threshold_))
  {
    RCLCPP_WARN(
      get_logger(),
      "Timestamp jitter is high; fixed time_offset may not be sufficient. cloud_std=%.6f odom_std=%.6f imu_std=%.6f threshold=%.6f",
      cloud_stats.stddev,
      odom_stats.stddev,
      imu_stats.stddev,
      delay_jitter_warning_threshold_);
  }
}

void DeskewNode::logFieldDetectionIfChanged(const PointCloudFieldHelper & helper)
{
  const std::string summary = helper.summary();
  if (summary == last_field_detection_summary_) {
    return;
  }

  last_field_detection_summary_ = summary;
  RCLCPP_INFO(get_logger(), "PointCloud2 field detection: %s", summary.c_str());
}

void DeskewNode::logCloudTiming(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const rclcpp::Time & actual_scan_start,
  const rclcpp::Time & actual_scan_end,
  double scan_duration,
  double min_point_offset,
  double max_point_offset) const
{
  RCLCPP_INFO(
    get_logger(),
    "cloud timing: header_stamp=%s actual_scan_start=%s actual_scan_end=%s scan_duration=%.9f min_point_offset=%.9f max_point_offset=%.9f",
    stampToString(rclcpp::Time(cloud.header.stamp)).c_str(),
    stampToString(actual_scan_start).c_str(),
    stampToString(actual_scan_end).c_str(),
    scan_duration,
    min_point_offset,
    max_point_offset);
}

void DeskewNode::logCustomMsgTiming(
  const livox_ros_driver2::msg::CustomMsg & cloud,
  const rclcpp::Time & cloud_stamp,
  const rclcpp::Time & actual_scan_start,
  const rclcpp::Time & actual_scan_end,
  double scan_duration,
  std::uint32_t min_point_offset,
  std::uint32_t max_point_offset) const
{
  RCLCPP_INFO(
    get_logger(),
    "CustomMsg timing: header_stamp=%s cloud_stamp=%s timebase=%llu timebase_type=%s offset_unit=%s actual_scan_start=%s actual_scan_end=%s scan_duration=%.9f min_offset_time=%u max_offset_time=%u point_num=%u points_size=%zu",
    stampToString(rclcpp::Time(cloud.header.stamp)).c_str(),
    stampToString(cloud_stamp).c_str(),
    static_cast<unsigned long long>(cloud.timebase),
    custom_msg_timebase_type_.c_str(),
    custom_msg_offset_unit_.c_str(),
    stampToString(actual_scan_start).c_str(),
    stampToString(actual_scan_end).c_str(),
    scan_duration,
    min_point_offset,
    max_point_offset,
    cloud.point_num,
    cloud.points.size());
}

void DeskewNode::logImuRotationDebug(
  const std::string & context,
  const rclcpp::Time & from,
  const rclcpp::Time & to,
  const Eigen::Quaterniond & q_from_to)
{
  if (!enable_debug_log_) {
    return;
  }

  const Eigen::Quaterniond q = q_from_to.normalized();
  const double yaw_delta = std::atan2(
    2.0 * (q.w() * q.z() + q.x() * q.y()),
    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    1000,
    "imu_rotation_debug context=%s from=%s to=%s dt=%.9f q=[%.9f, %.9f, %.9f, %.9f] yaw_delta=%.9f rad imu_rotation_sign=%.1f gyro_bias=[%.9f, %.9f, %.9f]",
    context.c_str(),
    stampToString(from).c_str(),
    stampToString(to).c_str(),
    (to - from).seconds(),
    q.w(),
    q.x(),
    q.y(),
    q.z(),
    yaw_delta,
    imu_rotation_sign_,
    gyro_bias_.x(),
    gyro_bias_.y(),
    gyro_bias_.z());
}

void DeskewNode::logPoseBufferStatus(const std::string & context) const
{
  const auto status = pose_buffer_.status();
  RCLCPP_INFO(
    get_logger(),
    "odom buffer status (%s): buffer_start=%s buffer_end=%s odom_sample_count=%zu interpolation_failed_count=%zu",
    context.c_str(),
    stampToString(status.start).c_str(),
    stampToString(status.end).c_str(),
    status.sample_count,
    interpolation_failed_count_);
}

void DeskewNode::logImuBufferStatus(const std::string & context) const
{
  const auto status = imu_buffer_.status();
  RCLCPP_INFO(
    get_logger(),
    "imu buffer status (%s): buffer_start=%s buffer_end=%s imu_sample_count=%zu imu_integration_failed_count=%zu gyro_bias=[%.9f, %.9f, %.9f] gyro_bias_estimated=%s",
    context.c_str(),
    stampToString(status.start).c_str(),
    stampToString(status.end).c_str(),
    status.sample_count,
    imu_integration_failed_count_,
    gyro_bias_.x(),
    gyro_bias_.y(),
    gyro_bias_.z(),
    gyro_bias_estimated_ ? "true" : "false");
}

std::string DeskewNode::stampToString(const rclcpp::Time & stamp)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(9) << stamp.seconds();
  return oss.str();
}

bool DeskewNode::isSupportedCloudInputType(const std::string & value)
{
  return value == "pointcloud2" || value == "custom_msg";
}

bool DeskewNode::isSupportedScanStampType(const std::string & value)
{
  return value == "scan_start" || value == "scan_end";
}

bool DeskewNode::isSupportedDeskewTargetTime(const std::string & value)
{
  return value == "scan_start" || value == "scan_mid" || value == "scan_end";
}

bool DeskewNode::isSupportedDeskewMethod(const std::string & value)
{
  return value == "odom" || value == "imu_only" || value == "odom_imu";
}

bool DeskewNode::isSupportedFallbackMethod(const std::string & value)
{
  return value == "none" || value == "odom" || value == "imu_only" || value == "linear";
}

bool DeskewNode::isSupportedRotationSource(const std::string & value)
{
  return value == "imu" || value == "odom" || value == "fused";
}

bool DeskewNode::isSupportedCustomMsgTimebaseType(const std::string & value)
{
  return value == "ros_header" || value == "custom_timebase";
}

bool DeskewNode::customMsgOffsetUnitScale(const std::string & value, double & scale)
{
  if (value == "second") {
    scale = 1.0;
    return true;
  }
  if (value == "millisecond") {
    scale = 1.0e-3;
    return true;
  }
  if (value == "microsecond") {
    scale = 1.0e-6;
    return true;
  }
  if (value == "nanosecond") {
    scale = 1.0e-9;
    return true;
  }
  scale = 0.0;
  return false;
}

double DeskewNode::odomTimeOffsetRuntime() const
{
  return odom_time_offset_config_ + odom_time_offset_auto_;
}

double DeskewNode::imuTimeOffsetRuntime() const
{
  return imu_time_offset_config_ + imu_time_offset_auto_;
}

}  // namespace mid360_deskew
