#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace mid360_deskew
{

class PointCloudFieldHelper
{
public:
  bool configure(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const std::string & preferred_time_field,
    const std::string & point_time_unit,
    std::string * error = nullptr);

  bool hasXYZ() const;
  bool hasPointTime() const;

  bool readXYZ(const std::uint8_t * point, double & x, double & y, double & z) const;
  bool writeXYZ(std::uint8_t * point, double x, double y, double z) const;
  bool readPointTime(const std::uint8_t * point, double & seconds) const;

  std::string detectedTimeFieldName() const;
  std::string detectedTimeFieldDatatypeName() const;
  std::string pointTimeUnit() const;
  std::string summary() const;

  static std::string datatypeName(std::uint8_t datatype);
  static std::size_t datatypeSize(std::uint8_t datatype);

private:
  struct FieldSpec
  {
    std::string name;
    std::uint32_t offset{0};
    std::uint8_t datatype{0};
    std::uint32_t count{1};
  };

  std::optional<FieldSpec> findField(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const std::string & name) const;

  bool readScalarAsDouble(
    const std::uint8_t * point,
    const FieldSpec & field,
    double & value) const;

  bool writeScalarFromDouble(
    std::uint8_t * point,
    const FieldSpec & field,
    double value) const;

  static bool hostIsBigEndian();
  static double unitScale(const std::string & point_time_unit, bool & ok);

  std::optional<FieldSpec> x_field_;
  std::optional<FieldSpec> y_field_;
  std::optional<FieldSpec> z_field_;
  std::optional<FieldSpec> time_field_;

  std::uint32_t point_step_{0};
  bool need_byte_swap_{false};
  std::string point_time_unit_;
  double point_time_unit_scale_{1.0};
};

}  // namespace mid360_deskew
