#include "global_occupancy_map/global_occupancy_map.hpp"

#include <cmath>
#include <limits>

#include <Eigen/Geometry>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace global_occupancy_map {

void GlobalOccupancyMap::configure(const ConfigType& config) {
    const bool resolution_changed = !configured_ || resolution_ != config.resolution;
    cloud_topic_ = config.cloud_topic;
    input_frame_ = config.input_frame;
    map_frame_ = config.map_frame;
    resolution_ = config.resolution;
    observation_policy_.max_buffer_size =
        config.max_buffer_size > 0 ? static_cast<std::size_t>(config.max_buffer_size) : 0U;
    observation_policy_.observation_keep_time_sec = config.observation_keep_time_sec;
    observation_policy_.source_timeout_sec = config.source_timeout_sec;
    observation_policy_.expected_update_rate_hz = config.expected_update_rate_hz;

    if (resolution_changed) {
        finenav_octomap::OctoMapConfig map_config;
        map_config.resolution = resolution_;
        map_.configure(map_config);
    }
    configured_ = true;
}

void GlobalOccupancyMap::selfManage(finenav::core::MapRuntime& runtime) {
    runtime.addObservationSource<sensor_msgs::msg::PointCloud2>(
        "point_cloud", cloud_topic_, rclcpp::SensorDataQoS(), observation_policy_,
        [this](const sensor_msgs::msg::PointCloud2& cloud,
               const rclcpp::Time&, const finenav::core::RobotState& state) {
            insertPointCloud(cloud, state);
        });
}

void GlobalOccupancyMap::insertPointCloud(
    const sensor_msgs::msg::PointCloud2& cloud,
    const finenav::core::RobotState& state) {
    if (!configured_) {
        RCLCPP_WARN(logger_,
                    "GlobalOccupancyMap received a point cloud before configure(); dropping it.");
        return;
    }

    const bool in_map_frame = cloud.header.frame_id == map_frame_;
    const bool in_input_frame = cloud.header.frame_id == input_frame_;
    if (!in_map_frame && !in_input_frame) {
        RCLCPP_WARN(
            logger_,
            "GlobalOccupancyMap: dropping cloud in unexpected frame '%s' "
            "(expected '%s' or '%s').",
            cloud.header.frame_id.c_str(), input_frame_.c_str(), map_frame_.c_str());
        return;
    }

    Eigen::Quaterniond orientation(
        state.pose.orientation.w, state.pose.orientation.x,
        state.pose.orientation.y, state.pose.orientation.z);
    if (orientation.norm() < std::numeric_limits<double>::epsilon()) {
        orientation = Eigen::Quaterniond::Identity();
    } else {
        orientation.normalize();
    }
    const Eigen::Vector3d translation(
        state.pose.position.x, state.pose.position.y, state.pose.position.z);

    try {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) ||
                !std::isfinite(*iter_z)) {
                continue;
            }
            Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);
            if (in_input_frame) {
                point = orientation * point + translation;
            }
            map_.insertPoint({point.x(), point.y(), point.z()});
        }
    } catch (const std::exception& error) {
        RCLCPP_WARN(logger_, "GlobalOccupancyMap: dropping malformed PointCloud2: %s",
                    error.what());
    }
}

bool GlobalOccupancyMap::isOccupied(
    const finenav::common::Position3D& position) const {
    return map_.isOccupied(position);
}

double GlobalOccupancyMap::getOccupancy(
    const finenav::common::Position3D& position) const {
    return map_.getOccupancy(position);
}

finenav::common::Region3D GlobalOccupancyMap::getBounds() const {
    return map_.getBounds();
}

double GlobalOccupancyMap::getResolution() const {
    return map_.getResolution();
}

std::size_t GlobalOccupancyMap::occupiedLeafCount() const {
    return map_.getNumLeafNodes();
}

}  // namespace global_occupancy_map
