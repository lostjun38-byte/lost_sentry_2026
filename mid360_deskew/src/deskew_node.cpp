#include "mid360_deskew/deskew_node.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
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
}  // namespace

DeskewNode::DeskewNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mid360_deskew_node", options),
  pose_buffer_(5.0, 0.05, false),
  imu_buffer_(5.0, 0.02)
{
  input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/mid360/points");
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

  if (filter_mode_ != "keep_size_nan") {
    RCLCPP_WARN(
      get_logger(),
      "Only filter_mode='keep_size_nan' is implemented. Requested '%s' will be treated as keep_size_nan.",
      filter_mode_.c_str());
    filter_mode_ = "keep_size_nan";
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

  pose_buffer_.configure(pose_buffer_duration_, max_allowed_time_gap_, allow_extrapolation_);
  imu_buffer_.configure(imu_buffer_duration_, max_allowed_imu_gap_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_cloud_topic_,
    rclcpp::SensorDataQoS());

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&DeskewNode::cloudCallback, this, std::placeholders::_1));

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
    "mid360_deskew_node started. input=%s output=%s odom=%s imu=%s fixed_frame=%s base_frame=%s lidar_frame=%s imu_frame=%s deskew_method=%s fallback_method=%s rotation_source=%s translation_source=%s",
    input_cloud_topic_.c_str(),
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
  failure_kind = FailureKind::Generic;

  if (method != "odom" && method != "linear" && method != "imu_only" && method != "odom_imu") {
    reason = "unsupported deskew method: " + method;
    return false;
  }

  const bool needs_odom = method == "odom" || method == "linear" || method == "odom_imu";
  const bool needs_imu = method == "imu_only" || method == "odom_imu";
  const auto poses = needs_odom ? pose_buffer_.snapshot() : std::vector<Pose>{};
  const auto imu_samples = needs_imu ? imu_buffer_.snapshot() : std::vector<ImuSample>{};

  // 时间偏移只在查询时应用，buffer 内保留消息原始 stamp，避免重复补偿。
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
      reason = "failed to interpolate odom pose at scan_start for linear fallback: " + lookup_reason;
      return false;
    }

    if (!lookup_odom_pose(timing.actual_scan_end, linear_end_pose, lookup_reason)) {
      logPoseBufferStatus("linear fallback scan_end pose lookup failed");
      reason = "failed to interpolate odom pose at scan_end for linear fallback: " + lookup_reason;
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

    t_source_lidar_ref = poseToIsometry(ref_pose) * t_base_lidar;
  }

  const Eigen::Isometry3d t_lidar_ref_source = t_source_lidar_ref.inverse();
  output = cloud;
  output.header.frame_id = lidar_frame_;
  output.header.stamp = timeToMsg(timing.t_ref);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::size_t valid_points = 0;
  std::size_t nan_points = 0;
  std::size_t range_filtered_points = 0;

  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      const std::size_t index =
        static_cast<std::size_t>(row) * static_cast<std::size_t>(cloud.width) +
        static_cast<std::size_t>(col);
      const std::uint8_t * input_point =
        cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step +
        static_cast<std::size_t>(col) * cloud.point_step;
      std::uint8_t * output_point =
        output.data.data() + static_cast<std::size_t>(row) * output.row_step +
        static_cast<std::size_t>(col) * output.point_step;

      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      if (!field_helper.readXYZ(input_point, x, y, z)) {
        reason = "failed to read x/y/z fields";
        failure_kind = FailureKind::Generic;
        return false;
      }

      const bool finite_point = std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
      if (!finite_point) {
        ++nan_points;
        output.is_dense = false;
        if (remove_nan_) {
          if (!field_helper.writeXYZ(output_point, nan, nan, nan)) {
            reason = "x/y/z fields must be FLOAT32 or FLOAT64 for writing";
            failure_kind = FailureKind::Generic;
            return false;
          }
        }
        continue;
      }

      const double range = std::sqrt(x * x + y * y + z * z);
      if (range < min_range_ || range > max_range_) {
        ++range_filtered_points;
        output.is_dense = false;
        if (!field_helper.writeXYZ(output_point, nan, nan, nan)) {
          reason = "x/y/z fields must be FLOAT32 or FLOAT64 for writing";
          failure_kind = FailureKind::Generic;
          return false;
        }
        continue;
      }

      const rclcpp::Time point_time =
        timing.actual_scan_start + secondsToDuration(point_offsets[index]);
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
        // q_ref_point 表示从参考时刻 lidar 坐标到当前点时刻 lidar 坐标的相对旋转。
        if (!integrate_imu(timing.t_ref, point_time, q_ref_point, imu_reason)) {
          logImuBufferStatus("imu_only point rotation integration failed");
          reason = "failed to integrate IMU rotation at point time: " + imu_reason;
          return false;
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

        const Eigen::Isometry3d t_odom_lidar_ref = poseToIsometry(ref_pose) * t_base_lidar;
        const Eigen::Isometry3d t_odom_lidar_i = poseToIsometry(point_pose) * t_base_lidar;
        const Eigen::Isometry3d t_rel_odom = t_odom_lidar_ref.inverse() * t_odom_lidar_i;

        // odom_imu 策略 A：相对平移取 odom，相对旋转优先取 IMU 高频积分。
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
      Eigen::Vector3d corrected_point = Eigen::Vector3d::Zero();
      // odom/linear 使用全局位姿链 T_ref^-1*T_i；imu_only/odom_imu 已直接构造 lidar_ref 下的相对变换。
      if (method == "imu_only" || method == "odom_imu") {
        corrected_point = t_source_lidar_i * raw_point;
      } else {
        corrected_point = t_lidar_ref_source * t_source_lidar_i * raw_point;
      }

      if (!field_helper.writeXYZ(
          output_point,
          corrected_point.x(),
          corrected_point.y(),
          corrected_point.z()))
      {
        reason = "x/y/z fields must be FLOAT32 or FLOAT64 for writing";
        failure_kind = FailureKind::Generic;
        return false;
      }

      ++valid_points;
    }
  }

  if (enable_debug_log_) {
    RCLCPP_INFO(
      get_logger(),
      "Deskew method '%s' completed: valid=%zu nan_or_inf=%zu range_filtered=%zu",
      method.c_str(),
      valid_points,
      nan_points,
      range_filtered_points);
  }

  return true;
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

void DeskewNode::recordCloudDelay(const sensor_msgs::msg::PointCloud2 & cloud)
{
  const rclcpp::Time now = this->now();
  const rclcpp::Time stamp(cloud.header.stamp);
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

double DeskewNode::odomTimeOffsetRuntime() const
{
  return odom_time_offset_config_ + odom_time_offset_auto_;
}

double DeskewNode::imuTimeOffsetRuntime() const
{
  return imu_time_offset_config_ + imu_time_offset_auto_;
}

}  // namespace mid360_deskew
