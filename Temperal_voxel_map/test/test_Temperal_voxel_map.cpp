#include <chrono>
#include <cmath>
#include <concepts>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "Temperal_voxel_map/Temperal_voxel_map.hpp"

template <typename T>
concept HasGlobalOccupancyApi = requires(
    const T& map, const finenav::common::Position3D& position) {
    { map.isOccupiedGlobal(position) } -> std::same_as<bool>;
    map.globalMap();
};

static_assert(!HasGlobalOccupancyApi<Temperal_voxel_map::TemperalVoxelMap>);

namespace {

sensor_msgs::msg::PointCloud2 onePointCloud(const std::string& frame,
                                            float x, float y, float z) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = frame;
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");
    *ix = x;
    *iy = y;
    *iz = z;
    return msg;
}

Temperal_voxel_map::TemperalVoxelMap configuredMap() {
    Temperal_voxel_map::TemperalVoxelMap map;
    Temperal_voxel_map::Params cfg;
    cfg.map_length = 10.0;
    cfg.resolution = 1.0;
    cfg.input_frame = "base_link";
    cfg.map_frame = "map";
    cfg.decay_time_sec = 1.0;
    map.configure(cfg);
    return map;
}

finenav::core::RobotState identityState() {
    finenav::core::RobotState state;
    state.pose.orientation.w = 1.0;
    return state;
}

TEST(TemperalVoxelMap, InsertsCloudAlreadyInMapFrame) {
    auto map = configuredMap();
    const auto now = Temperal_voxel_map::TemperalVoxelMap::Clock::time_point{};
    map.insertPointCloud(onePointCloud("map", 1.0F, 2.0F, 3.0F), identityState(), now);

    EXPECT_TRUE(map.isOccupied({1.0, 2.0, 3.0}));
    EXPECT_EQ(map.occupiedCellCount(), 1U);
}

TEST(TemperalVoxelMap, TransformsCloudFromInputFrameUsingRobotStatePose) {
    auto map = configuredMap();
    auto state = identityState();
    state.pose.position.x = 2.0;
    state.pose.position.y = 3.0;
    state.pose.orientation.w = std::sqrt(0.5);
    state.pose.orientation.z = std::sqrt(0.5);
    const auto now = Temperal_voxel_map::TemperalVoxelMap::Clock::time_point{};

    map.insertPointCloud(onePointCloud("base_link", 1.0F, 0.0F, 0.0F), state, now);

    EXPECT_TRUE(map.isOccupied({2.0, 4.0, 0.0}));
}

TEST(TemperalVoxelMap, RejectsUnexpectedCloudFrame) {
    auto map = configuredMap();
    map.insertPointCloud(onePointCloud("camera_link", 1.0F, 2.0F, 3.0F), identityState(),
                         Temperal_voxel_map::TemperalVoxelMap::Clock::time_point{});

    EXPECT_EQ(map.occupiedCellCount(), 0U);
}

TEST(TemperalVoxelMap, PrunesCellsPastConfiguredLifetime) {
    auto map = configuredMap();
    const auto t0 = Temperal_voxel_map::TemperalVoxelMap::Clock::time_point{};
    map.insertPointCloud(onePointCloud("map", 1.0F, 2.0F, 3.0F), identityState(), t0);

    map.pruneExpiredCells(t0 + std::chrono::milliseconds(1100));

    EXPECT_FALSE(map.isOccupied({1.0, 2.0, 3.0}));
    EXPECT_EQ(map.occupiedCellCount(), 0U);
}

TEST(TemperalVoxelMap, UpdatesActiveIndicesWhenWindowRolls) {
    auto map = configuredMap();
    const auto t0 = Temperal_voxel_map::TemperalVoxelMap::Clock::time_point{};
    map.insertPointCloud(onePointCloud("map", 1.0F, 0.0F, 0.0F), identityState(), t0);

    map.shiftWindowTo({3.0, 0.0, 0.0});
    EXPECT_TRUE(map.isOccupied({1.0, 0.0, 0.0}));
    EXPECT_EQ(map.occupiedCellCount(), 1U);

    map.pruneExpiredCells(t0 + std::chrono::milliseconds(1100));
    EXPECT_EQ(map.occupiedCellCount(), 0U);
}

}  // namespace
