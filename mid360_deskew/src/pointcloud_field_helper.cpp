#include "mid360_deskew/pointcloud_field_helper.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <sstream>
#include <vector>

namespace mid360_deskew
{

namespace
{
// sensor_msgs::msg::PointField 没有标准 UINT64 枚举；部分 Livox 点云会用 9 表示 64 位时间字段。
constexpr std::uint8_t DATATYPE_UINT64_NONSTANDARD = 9;

template<typename T>
T readValue(const std::uint8_t * data, bool need_byte_swap)
{
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), data, sizeof(T));
  if (need_byte_swap) {
    std::reverse(bytes.begin(), bytes.end());
  }
  T value{};
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}

template<typename T>
void writeValue(std::uint8_t * data, T value, bool need_byte_swap)
{
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
  if (need_byte_swap) {
    std::reverse(bytes.begin(), bytes.end());
  }
  std::memcpy(data, bytes.data(), sizeof(T));
}
}  // namespace

bool PointCloudFieldHelper::configure(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & preferred_time_field,
  const std::string & point_time_unit,
  std::string * error)
{
  point_step_ = cloud.point_step;
  need_byte_swap_ = cloud.is_bigendian != hostIsBigEndian();
  point_time_unit_ = point_time_unit;

  bool unit_ok = false;
  point_time_unit_scale_ = unitScale(point_time_unit, unit_ok);
  if (!unit_ok) {
    if (error) {
      *error = "unsupported point_time_unit: " + point_time_unit;
    }
    return false;
  }

  x_field_ = findField(cloud, "x");
  y_field_ = findField(cloud, "y");
  z_field_ = findField(cloud, "z");

  time_field_.reset();
  std::vector<std::string> candidates;
  if (!preferred_time_field.empty()) {
    candidates.push_back(preferred_time_field);
  }
  // 用户指定字段优先；不存在时自动尝试常见 Livox/MID360 点时间字段。
  for (const auto & name : {"offset_time", "time", "timestamp", "t"}) {
    if (std::find(candidates.begin(), candidates.end(), name) == candidates.end()) {
      candidates.push_back(name);
    }
  }

  for (const auto & name : candidates) {
    auto field = findField(cloud, name);
    if (field.has_value()) {
      time_field_ = field;
      break;
    }
  }

  return true;
}

bool PointCloudFieldHelper::hasXYZ() const
{
  return x_field_.has_value() && y_field_.has_value() && z_field_.has_value();
}

bool PointCloudFieldHelper::hasPointTime() const
{
  return time_field_.has_value();
}

bool PointCloudFieldHelper::readXYZ(
  const std::uint8_t * point,
  double & x,
  double & y,
  double & z) const
{
  if (!hasXYZ()) {
    return false;
  }

  return readScalarAsDouble(point, *x_field_, x) &&
         readScalarAsDouble(point, *y_field_, y) &&
         readScalarAsDouble(point, *z_field_, z);
}

bool PointCloudFieldHelper::writeXYZ(
  std::uint8_t * point,
  double x,
  double y,
  double z) const
{
  if (!hasXYZ()) {
    return false;
  }

  return writeScalarFromDouble(point, *x_field_, x) &&
         writeScalarFromDouble(point, *y_field_, y) &&
         writeScalarFromDouble(point, *z_field_, z);
}

bool PointCloudFieldHelper::readPointTime(const std::uint8_t * point, double & seconds) const
{
  if (!time_field_.has_value()) {
    return false;
  }

  double raw_value = 0.0;
  if (!readScalarAsDouble(point, *time_field_, raw_value)) {
    return false;
  }

  seconds = raw_value * point_time_unit_scale_;
  return true;
}

std::string PointCloudFieldHelper::detectedTimeFieldName() const
{
  return time_field_.has_value() ? time_field_->name : "";
}

std::string PointCloudFieldHelper::detectedTimeFieldDatatypeName() const
{
  return time_field_.has_value() ? datatypeName(time_field_->datatype) : "";
}

std::string PointCloudFieldHelper::pointTimeUnit() const
{
  return point_time_unit_;
}

std::string PointCloudFieldHelper::summary() const
{
  std::ostringstream oss;
  oss << "fields: x=" << (x_field_.has_value() ? "yes" : "no")
      << ", y=" << (y_field_.has_value() ? "yes" : "no")
      << ", z=" << (z_field_.has_value() ? "yes" : "no")
      << ", point_time_field=";
  if (time_field_.has_value()) {
    oss << time_field_->name << " (" << datatypeName(time_field_->datatype) << ")";
  } else {
    oss << "missing";
  }
  oss << ", point_time_unit=" << point_time_unit_;
  return oss.str();
}

std::string PointCloudFieldHelper::datatypeName(std::uint8_t datatype)
{
  using sensor_msgs::msg::PointField;
  switch (datatype) {
    case PointField::INT8:
      return "INT8";
    case PointField::UINT8:
      return "UINT8";
    case PointField::INT16:
      return "INT16";
    case PointField::UINT16:
      return "UINT16";
    case PointField::INT32:
      return "INT32";
    case PointField::UINT32:
      return "UINT32";
    case PointField::FLOAT32:
      return "FLOAT32";
    case PointField::FLOAT64:
      return "FLOAT64";
    case DATATYPE_UINT64_NONSTANDARD:
      return "UINT64";
    default:
      return "UNKNOWN(" + std::to_string(datatype) + ")";
  }
}

std::size_t PointCloudFieldHelper::datatypeSize(std::uint8_t datatype)
{
  using sensor_msgs::msg::PointField;
  switch (datatype) {
    case PointField::INT8:
    case PointField::UINT8:
      return 1;
    case PointField::INT16:
    case PointField::UINT16:
      return 2;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
      return 4;
    case PointField::FLOAT64:
    case DATATYPE_UINT64_NONSTANDARD:
      return 8;
    default:
      return 0;
  }
}

std::optional<PointCloudFieldHelper::FieldSpec> PointCloudFieldHelper::findField(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name) const
{
  for (const auto & field : cloud.fields) {
    if (field.name != name) {
      continue;
    }

    const std::size_t element_size = datatypeSize(field.datatype);
    const std::uint32_t count = field.count == 0 ? 1 : field.count;
    if (element_size == 0) {
      return std::nullopt;
    }
    if (field.offset + element_size * count > cloud.point_step) {
      return std::nullopt;
    }

    return FieldSpec{field.name, field.offset, field.datatype, count};
  }
  return std::nullopt;
}

bool PointCloudFieldHelper::readScalarAsDouble(
  const std::uint8_t * point,
  const FieldSpec & field,
  double & value) const
{
  using sensor_msgs::msg::PointField;
  if (field.offset + datatypeSize(field.datatype) > point_step_) {
    return false;
  }

  const std::uint8_t * data = point + field.offset;
  switch (field.datatype) {
    case PointField::INT8:
      value = static_cast<double>(readValue<std::int8_t>(data, need_byte_swap_));
      return true;
    case PointField::UINT8:
      value = static_cast<double>(readValue<std::uint8_t>(data, need_byte_swap_));
      return true;
    case PointField::INT16:
      value = static_cast<double>(readValue<std::int16_t>(data, need_byte_swap_));
      return true;
    case PointField::UINT16:
      value = static_cast<double>(readValue<std::uint16_t>(data, need_byte_swap_));
      return true;
    case PointField::INT32:
      value = static_cast<double>(readValue<std::int32_t>(data, need_byte_swap_));
      return true;
    case PointField::UINT32:
      value = static_cast<double>(readValue<std::uint32_t>(data, need_byte_swap_));
      return true;
    case PointField::FLOAT32:
      value = static_cast<double>(readValue<float>(data, need_byte_swap_));
      return true;
    case PointField::FLOAT64:
      value = readValue<double>(data, need_byte_swap_);
      return true;
    case DATATYPE_UINT64_NONSTANDARD:
      value = static_cast<double>(readValue<std::uint64_t>(data, need_byte_swap_));
      return true;
    default:
      return false;
  }
}

bool PointCloudFieldHelper::writeScalarFromDouble(
  std::uint8_t * point,
  const FieldSpec & field,
  double value) const
{
  using sensor_msgs::msg::PointField;
  if (field.offset + datatypeSize(field.datatype) > point_step_) {
    return false;
  }

  std::uint8_t * data = point + field.offset;
  switch (field.datatype) {
    case PointField::FLOAT32:
      writeValue<float>(data, static_cast<float>(value), need_byte_swap_);
      return true;
    case PointField::FLOAT64:
      writeValue<double>(data, value, need_byte_swap_);
      return true;
    default:
      return false;
  }
}

bool PointCloudFieldHelper::hostIsBigEndian()
{
  const std::uint16_t value = 0x0102;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 0x01;
}

double PointCloudFieldHelper::unitScale(const std::string & point_time_unit, bool & ok)
{
  ok = true;
  if (point_time_unit == "second") {
    return 1.0;
  }
  if (point_time_unit == "millisecond") {
    return 1.0e-3;
  }
  if (point_time_unit == "microsecond") {
    return 1.0e-6;
  }
  if (point_time_unit == "nanosecond") {
    return 1.0e-9;
  }

  ok = false;
  return 1.0;
}

}  // namespace mid360_deskew
