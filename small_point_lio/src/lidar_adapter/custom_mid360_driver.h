/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "base_lidar.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace small_point_lio {

    class CustomMid360DriverAdapter : public LidarAdapterBase {
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;

    public:
        inline void setup_subscription(rclcpp::Node *node, const std::string &topic, std::function<void(const std::vector<common::Point> &)> callback) override {
            subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                    topic,
                    rclcpp::SensorDataQoS(),
                    [callback](const sensor_msgs::msg::PointCloud2 &msg) {
                        sensor_msgs::PointCloud2ConstIterator<float> out_x(msg, "x");
                        sensor_msgs::PointCloud2ConstIterator<float> out_y(msg, "y");
                        sensor_msgs::PointCloud2ConstIterator<float> out_z(msg, "z");

                        // [修复1] 检查时间字段存在性和类型
                        bool has_timestamp = false;
                        bool has_time = false;
                        uint8_t timestamp_datatype = 0;
                        uint8_t time_datatype = 0;
                        for (const auto &field: msg.fields) {
                            if (field.name == "timestamp") {
                                has_timestamp = true;
                                timestamp_datatype = field.datatype;
                            }
                            if (field.name == "time") {
                                has_time = true;
                                time_datatype = field.datatype;
                            }
                        }

                        // 获取消息头的基准时间（绝对时间，单位：秒）
                        double header_time = rclcpp::Time(msg.header.stamp).seconds();

                        size_t size = msg.width * msg.height;
                        std::vector<common::Point> pointcloud;
                        pointcloud.reserve(size);

                        if (has_timestamp && timestamp_datatype == sensor_msgs::msg::PointField::FLOAT64) {
                            // timestamp 字段是 double 类型
                            sensor_msgs::PointCloud2ConstIterator<double> out_timestamp(msg, "timestamp");
                            for (size_t i = 0; i < size; ++i) {
                                // [修复2] 添加点坐标有效性检查
                                if (std::isfinite(*out_x) && std::isfinite(*out_y) && std::isfinite(*out_z)) {
                                    common::Point new_point;
                                    new_point.position << *out_x, *out_y, *out_z;
                                    double time_value = *out_timestamp;
                                    // [修复3] 智能判断时间戳单位
                                    if (time_value > 1e9) {
                                        // 纳秒级绝对时间
                                        new_point.timestamp = time_value * 1e-9;
                                    } else if (time_value > 1e6) {
                                        // 已经是秒级绝对时间
                                        new_point.timestamp = time_value;
                                    } else {
                                        // 相对时间（秒），加上 header 基准
                                        new_point.timestamp = header_time + time_value;
                                    }
                                    // [修复4] 验证时间戳有效性
                                    if (std::isfinite(new_point.timestamp) && new_point.timestamp > 0) {
                                        pointcloud.push_back(new_point);
                                    }
                                }
                                ++out_x;
                                ++out_y;
                                ++out_z;
                                ++out_timestamp;
                            }
                        } else if (has_time) {
                            // time 字段处理
                            sensor_msgs::PointCloud2ConstIterator<float> out_time(msg, "time");
                            for (size_t i = 0; i < size; ++i) {
                                if (std::isfinite(*out_x) && std::isfinite(*out_y) && std::isfinite(*out_z)) {
                                    common::Point new_point;
                                    new_point.position << *out_x, *out_y, *out_z;
                                    double relative_time = static_cast<double>(*out_time);
                                    // [修复5] 验证相对时间合理性（一帧内通常 < 0.2 秒）
                                    if (std::isfinite(relative_time) && relative_time >= 0.0 && relative_time < 1.0) {
                                        new_point.timestamp = header_time + relative_time;
                                    } else {
                                        new_point.timestamp = header_time;
                                    }
                                    pointcloud.push_back(new_point);
                                }
                                ++out_x;
                                ++out_y;
                                ++out_z;
                                ++out_time;
                            }
                        } else {
                            // [修复6] 没有时间字段，使用帧头时间
                            for (size_t i = 0; i < size; ++i) {
                                if (std::isfinite(*out_x) && std::isfinite(*out_y) && std::isfinite(*out_z)) {
                                    common::Point new_point;
                                    new_point.position << *out_x, *out_y, *out_z;
                                    new_point.timestamp = header_time;
                                    pointcloud.push_back(new_point);
                                }
                                ++out_x;
                                ++out_y;
                                ++out_z;
                            }
                        }

                        callback(pointcloud);
                    });
        }
        // inline void setup_subscription(rclcpp::Node *node, const std::string &topic, std::function<void(const std::vector<common::Point> &)> callback) override {
        //     subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        //             topic,
        //             rclcpp::SensorDataQoS(),
        //             [callback](const sensor_msgs::msg::PointCloud2 &msg) {
        //                 sensor_msgs::PointCloud2ConstIterator<float> out_x(msg, "x");
        //                 sensor_msgs::PointCloud2ConstIterator<float> out_y(msg, "y");
        //                 sensor_msgs::PointCloud2ConstIterator<float> out_z(msg, "z");
        //                 sensor_msgs::PointCloud2ConstIterator<double> out_timestamp(msg, "timestamp");
        //                 size_t size = msg.width * msg.height;
        //                 std::vector<common::Point> pointcloud;
        //                 pointcloud.reserve(size);
        //                 for (size_t i = 0; i < size; ++i) {
        //                     common::Point new_point;
        //                     new_point.position << *out_x, *out_y, *out_z;
        //                     new_point.timestamp = *out_timestamp;
        //                     pointcloud.push_back(new_point);
        //                     ++out_x;
        //                     ++out_y;
        //                     ++out_z;
        //                     ++out_timestamp;
        //                 }
        //                 callback(pointcloud);
        //             });
        // }

        // 替换原来的 setup_subscription 函数
        // inline void setup_subscription(rclcpp::Node *node, const std::string &topic, std::function<void(const std::vector<common::Point> &)> callback) override {
        //     subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        //             topic,
        //             rclcpp::SensorDataQoS(),
        //             [callback](const sensor_msgs::msg::PointCloud2 &msg) {
        //                 sensor_msgs::PointCloud2ConstIterator<float> out_x(msg, "x");
        //                 sensor_msgs::PointCloud2ConstIterator<float> out_y(msg, "y");
        //                 sensor_msgs::PointCloud2ConstIterator<float> out_z(msg, "z");

        //                 bool has_timestamp = false;
        //                 bool has_time = false;
        //                 for (const auto &field: msg.fields) {
        //                     if (field.name == "timestamp") has_timestamp = true;
        //                     if (field.name == "time") has_time = true;
        //                 }

        //                 // --- 新增：获取消息头的基准时间 ---
        //                 double header_time = rclcpp::Time(msg.header.stamp).seconds();
        //                 // ------------------------------

        //                 size_t size = msg.width * msg.height;
        //                 std::vector<common::Point> pointcloud;
        //                 pointcloud.reserve(size);

        //                 if (has_timestamp) {
        //                     // Livox CustomMsg 通常已经是绝对时间，或者根据协议判断
        //                     // 如果 Livox 发的是绝对时间，这里不用动；如果是相对的，也需要加 header_time
        //                     // 通常 Livox 的 timestamp 字段是绝对时间（纳秒），但这里如果是 PointCloud2 转换来的，要小心
        //                     sensor_msgs::PointCloud2ConstIterator<double> out_timestamp(msg, "timestamp");
        //                     for (size_t i = 0; i < size; ++i) {
        //                         common::Point new_point;
        //                         new_point.position << *out_x, *out_y, *out_z;
        //                         new_point.timestamp = *out_timestamp;// 假设这里原本就是绝对时间
        //                         pointcloud.push_back(new_point);
        //                         ++out_x;
        //                         ++out_y;
        //                         ++out_z;
        //                         ++out_timestamp;
        //                     }
        //                 } else if (has_time) {
        //                     // --- 修改重点：处理 "time" 字段 ---
        //                     sensor_msgs::PointCloud2ConstIterator<float> out_time(msg, "time");
        //                     for (size_t i = 0; i < size; ++i) {
        //                         common::Point new_point;
        //                         new_point.position << *out_x, *out_y, *out_z;

        //                         // 修正：绝对时间 = 帧头时间 + 点的相对偏移时间
        //                         new_point.timestamp = header_time + static_cast<double>(*out_time);

        //                         pointcloud.push_back(new_point);
        //                         ++out_x;
        //                         ++out_y;
        //                         ++out_z;
        //                         ++out_time;
        //                     }
        //                 } else {
        //                     // 没有时间字段，所有点都认为是帧头时间
        //                     for (size_t i = 0; i < size; ++i) {
        //                         common::Point new_point;
        //                         new_point.position << *out_x, *out_y, *out_z;
        //                         new_point.timestamp = header_time;// 使用帧头时间
        //                         pointcloud.push_back(new_point);
        //                         ++out_x;
        //                         ++out_y;
        //                         ++out_z;
        //                     }
        //                 }

        //                 callback(pointcloud);
        //             });
        // }
    };

}// namespace small_point_lio
