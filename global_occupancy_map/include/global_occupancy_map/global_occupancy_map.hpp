#pragma once

#include <cstddef>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "finenav_core/plugin_map.hpp"
#include "finenav_octomap/octomap.hpp"
#include "global_occupancy_map/global_occupancy_map_params.hpp"

namespace global_occupancy_map {

class GlobalOccupancyMap {
public:
    using ConfigType = Params;
    using DataType = bool;

    void configure(const ConfigType& config);
    void selfManage(finenav::core::MapRuntime& runtime);
    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }

    void insertPointCloud(const sensor_msgs::msg::PointCloud2& cloud,
                          const finenav::core::RobotState& state);

    bool isOccupied(const finenav::common::Position3D& position) const;
    double getOccupancy(const finenav::common::Position3D& position) const;
    finenav::common::Region3D getBounds() const;
    double getResolution() const;
    std::size_t occupiedLeafCount() const;

private:
    finenav_octomap::OctoMap map_;
    std::string cloud_topic_{"/points"};
    std::string input_frame_{"base_link"};
    std::string map_frame_{"map"};
    double resolution_{0.2};
    finenav::core::ObservationBufferPolicy observation_policy_{};
    rclcpp::Logger logger_{rclcpp::get_logger("GlobalOccupancyMap")};
    bool configured_{false};
};

}  // namespace global_occupancy_map
