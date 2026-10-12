#include <stdexcept>

#include <gtest/gtest.h>

#include "finenav_octomap/octomap.hpp"

namespace {

finenav_octomap::OctoMap configuredMap(double resolution = 0.1) {
    finenav_octomap::OctoMap map;
    finenav_octomap::OctoMapConfig config;
    config.resolution = resolution;
    map.configure(config);
    return map;
}

TEST(OctoMap, InsertsAndQueriesOccupancy) {
    auto map = configuredMap();

    map.insertPoint({1.0, 2.0, 3.0}, true);

    EXPECT_TRUE(map.isOccupied({1.0, 2.0, 3.0}));
    EXPECT_FALSE(map.isOccupied({0.0, 0.0, 0.0}));
    EXPECT_GT(map.getNumLeafNodes(), 0U);
}

TEST(OctoMap, ClearResetsTree) {
    auto map = configuredMap();

    map.insertPoint({1.0, 2.0, 3.0});
    EXPECT_GT(map.getNumLeafNodes(), 0U);

    map.clear();
    EXPECT_EQ(map.getNumLeafNodes(), 0U);
}

TEST(OctoMap, OccupancyProbabilityGrowsWithRepeatedHits) {
    auto map = configuredMap();

    map.insertPoint({0.0, 0.0, 0.0}, true);
    const double first = map.getOccupancy({0.0, 0.0, 0.0});
    map.insertPoint({0.0, 0.0, 0.0}, true);
    const double second = map.getOccupancy({0.0, 0.0, 0.0});

    EXPECT_GT(second, first);
}

TEST(OctoMap, FreeUpdateLowersOccupancy) {
    auto map = configuredMap();

    map.insertPoint({0.0, 0.0, 0.0}, true);
    const double occupied = map.getOccupancy({0.0, 0.0, 0.0});

    map.insertPoint({0.0, 0.0, 0.0}, false);
    const double after_free = map.getOccupancy({0.0, 0.0, 0.0});

    EXPECT_LT(after_free, occupied);
}

TEST(OctoMap, ThrowsOnNonPositiveResolution) {
    finenav_octomap::OctoMap map;
    finenav_octomap::OctoMapConfig config;
    config.resolution = 0.0;
    EXPECT_THROW(map.configure(config), std::invalid_argument);
}

}  // namespace
