#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "global_occupancy_map/global_occupancy_map.hpp"

template <typename T>
concept HasPruneApi = requires(T& map) {
    map.pruneExpiredCells();
};

static_assert(finenav::core::Map<global_occupancy_map::GlobalOccupancyMap>);
static_assert(!finenav::core::ShiftableMap<global_occupancy_map::GlobalOccupancyMap>);
static_assert(!HasPruneApi<global_occupancy_map::GlobalOccupancyMap>);

namespace {

sensor_msgs::msg::PointCloud2 onePointCloud(
    const std::string& frame, float x, float y, float z) {
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

global_occupancy_map::GlobalOccupancyMap configuredMap() {
    global_occupancy_map::GlobalOccupancyMap map;
    global_occupancy_map::Params config;
    config.resolution = 1.0;
    config.input_frame = "base_link";
    config.map_frame = "map";
    map.configure(config);
    return map;
}

finenav::core::RobotState identityState() {
    finenav::core::RobotState state;
    state.pose.orientation.w = 1.0;
    return state;
}

TEST(GlobalOccupancyMap, InsertsCloudAlreadyInMapFrame) {
    auto map = configuredMap();
    map.insertPointCloud(onePointCloud("map", 8.0F, -2.0F, 1.0F), identityState());

    EXPECT_TRUE(map.isOccupied({8.0, -2.0, 1.0}));
    EXPECT_EQ(map.occupiedLeafCount(), 1U);
}

TEST(GlobalOccupancyMap, TransformsCloudFromInputFrameUsingRobotStatePose) {
    auto map = configuredMap();
    auto state = identityState();
    state.pose.position.x = 2.0;
    state.pose.position.y = 3.0;
    state.pose.orientation.w = std::sqrt(0.5);
    state.pose.orientation.z = std::sqrt(0.5);

    map.insertPointCloud(onePointCloud("base_link", 1.0F, 0.0F, 0.0F), state);

    EXPECT_TRUE(map.isOccupied({2.0, 4.0, 0.0}));
}

TEST(GlobalOccupancyMap, RejectsUnexpectedCloudFrame) {
    auto map = configuredMap();
    map.insertPointCloud(
        onePointCloud("camera_link", 1.0F, 2.0F, 3.0F), identityState());

    EXPECT_EQ(map.occupiedLeafCount(), 0U);
}

TEST(GlobalOccupancyMap, SkipsNonFinitePoints) {
    auto map = configuredMap();
    map.insertPointCloud(
        onePointCloud("map", std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F),
        identityState());

    EXPECT_EQ(map.occupiedLeafCount(), 0U);
}

TEST(GlobalOccupancyMap, RejectsMalformedCloud) {
    auto map = configuredMap();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "map";

    map.insertPointCloud(cloud, identityState());

    EXPECT_EQ(map.occupiedLeafCount(), 0U);
}

TEST(GlobalOccupancyMap, AccumulatesDistantOccupiedEndpoints) {
    auto map = configuredMap();
    map.insertPointCloud(onePointCloud("map", -100.0F, 0.0F, 0.0F), identityState());
    map.insertPointCloud(onePointCloud("map", 100.0F, 0.0F, 0.0F), identityState());

    EXPECT_TRUE(map.isOccupied({-100.0, 0.0, 0.0}));
    EXPECT_TRUE(map.isOccupied({100.0, 0.0, 0.0}));
    EXPECT_GE(map.occupiedLeafCount(), 2U);
}

}  // namespace
