#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

#include "grid_map/grid_map.hpp"

namespace {

grid_map::GridMapConfig validConfig() {
    grid_map::GridMapConfig config;
    config.length = {5.0, 5.0, 5.0};
    config.resolution = 1.0;
    config.origin = {0.0, 0.0, 0.0};
    return config;
}

void expectCenter(const grid_map::GridMap<int>& grid, double x, double y, double z) {
    const auto center = grid.getWindowCenter();
    EXPECT_DOUBLE_EQ(center.x, x);
    EXPECT_DOUBLE_EQ(center.y, y);
    EXPECT_DOUBLE_EQ(center.z, z);
}

TEST(GridMap, RejectsNonFiniteConfigurationValues) {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    for (const double invalid : {infinity, -infinity, nan}) {
        {
            grid_map::GridMap<int> grid;
            auto config = validConfig();
            config.resolution = invalid;
            EXPECT_THROW(grid.configure(config), std::invalid_argument);
        }
        {
            grid_map::GridMap<int> grid;
            auto config = validConfig();
            config.length.x = invalid;
            EXPECT_THROW(grid.configure(config), std::invalid_argument);
        }
        {
            grid_map::GridMap<int> grid;
            auto config = validConfig();
            config.origin.z = invalid;
            EXPECT_THROW(grid.configure(config), std::invalid_argument);
        }
    }
}

TEST(GridMap, RejectsNonPositiveLengthAndResolution) {
    for (const double invalid : {0.0, -1.0}) {
        {
            grid_map::GridMap<int> grid;
            auto config = validConfig();
            config.resolution = invalid;
            EXPECT_THROW(grid.configure(config), std::invalid_argument);
        }
        {
            grid_map::GridMap<int> grid;
            auto config = validConfig();
            config.length.y = invalid;
            EXPECT_THROW(grid.configure(config), std::invalid_argument);
        }
    }
}

TEST(GridMap, RejectsAxisCellCountOverflowWithoutChangingExistingMap) {
    grid_map::GridMap<int> grid;
    auto initial = validConfig();
    initial.origin = {1.0, 2.0, 3.0};
    grid.configure(initial);
    grid.at({0, 0, 0}) = 17;

    auto oversized = validConfig();
    oversized.length.x = static_cast<double>(std::numeric_limits<int>::max()) + 1.0;
    oversized.origin = {9.0, 9.0, 9.0};

    EXPECT_THROW(grid.configure(oversized), std::length_error);
    expectCenter(grid, 1.0, 2.0, 3.0);
    EXPECT_EQ(grid.at({0, 0, 0}), 17);
}

TEST(GridMap, RejectsTotalCellCountOverflowWithoutChangingExistingMap) {
    grid_map::GridMap<int> grid;
    auto initial = validConfig();
    initial.origin = {1.0, 2.0, 3.0};
    grid.configure(initial);
    grid.at({0, 0, 0}) = 23;

    auto oversized = validConfig();
    oversized.length = {3'000'000.0, 3'000'000.0, 3'000'000.0};
    oversized.origin = {8.0, 8.0, 8.0};

    EXPECT_THROW(grid.configure(oversized), std::length_error);
    expectCenter(grid, 1.0, 2.0, 3.0);
    EXPECT_EQ(grid.at({0, 0, 0}), 23);
}

TEST(GridMap, RejectsNonFiniteWorldPositionsWithoutMovingTheWindow) {
    grid_map::GridMap<int> grid;
    grid.configure(validConfig());

    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const finenav::common::Position3D invalid_positions[] = {
        {nan, 0.0, 0.0},
        {0.0, infinity, 0.0},
        {0.0, 0.0, -infinity},
    };

    for (const auto& position : invalid_positions) {
        EXPECT_FALSE(grid.isInside(position));
        EXPECT_THROW(grid.getIndex(position), std::invalid_argument);
        EXPECT_THROW(grid.atPosition(position), std::invalid_argument);
        EXPECT_THROW(grid.shiftWindowTo(position), std::invalid_argument);
        expectCenter(grid, 0.0, 0.0, 0.0);
    }
}

TEST(GridMap, ConvertsWorldPositionsToRoundedLogicalIndices) {
    grid_map::GridMap<int> grid;
    grid_map::GridMapConfig config;
    config.length = {5.0, 5.0, 5.0};
    config.resolution = 1.0;
    config.origin = {0.0, 0.0, 0.0};
    grid.configure(config);

    const auto index = grid.getIndex(finenav::common::Position3D{0.49, -0.49, 1.51});
    EXPECT_EQ(index.x, 0);
    EXPECT_EQ(index.y, 0);
    EXPECT_EQ(index.z, 2);
    EXPECT_TRUE(grid.isInside(finenav::common::Position3D{2.0, -2.0, 2.0}));
    EXPECT_FALSE(grid.isInside(finenav::common::Position3D{3.0, 0.0, 0.0}));
}

TEST(GridMap, ShiftRetainsOverlapAndClearsCellsLeavingWindow) {
    grid_map::GridMap<int> grid;
    grid_map::GridMapConfig config;
    config.length = {5.0, 5.0, 5.0};
    config.resolution = 1.0;
    config.origin = {0.0, 0.0, 0.0};
    grid.configure(config);
    grid.atPosition(finenav::common::Position3D{1.0, 0.0, 0.0}) = 7;
    grid.atPosition(finenav::common::Position3D{0.0, 0.0, 0.0}) = 4;

    grid.shiftWindowTo(finenav::common::Position3D{3.0, 0.0, 0.0});

    const auto center = grid.getWindowCenter();
    EXPECT_DOUBLE_EQ(center.x, 3.0);
    EXPECT_DOUBLE_EQ(center.y, 0.0);
    EXPECT_DOUBLE_EQ(center.z, 0.0);
    EXPECT_TRUE(grid.isInside(finenav::common::Position3D{1.0, 0.0, 0.0}));
    EXPECT_EQ(grid.atPosition(finenav::common::Position3D{1.0, 0.0, 0.0}), 7);
    EXPECT_FALSE(grid.isInside(finenav::common::Position3D{0.0, 0.0, 0.0}));
}

TEST(GridMap, MultiAxisShiftRetainsOnlyTheOverlappingCells) {
    grid_map::GridMap<int> grid;
    auto config = validConfig();
    config.length = {3.0, 3.0, 3.0};
    grid.configure(config);
    grid.at({-1, -1, -1}) = 11;
    grid.at({0, 0, 0}) = 22;
    grid.at({1, 1, 1}) = 33;

    std::vector<grid_map::GridIndex> removed;
    ASSERT_TRUE(grid.shiftWindowTo({1.0, 1.0, 1.0}, &removed));

    expectCenter(grid, 1.0, 1.0, 1.0);
    EXPECT_EQ(grid.at({-1, -1, -1}), 22);
    EXPECT_EQ(grid.at({0, 0, 0}), 33);
    EXPECT_EQ(grid.at({1, 1, 1}), 0);
    EXPECT_EQ(removed.size(), 19U);
}

TEST(GridMap, ShiftBeyondTheWindowClearsEveryCellAndReportsEveryOldIndex) {
    grid_map::GridMap<int> grid;
    auto config = validConfig();
    config.length = {3.0, 3.0, 3.0};
    grid.configure(config);

    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            for (int z = -1; z <= 1; ++z) {
                grid.at({x, y, z}) = 1;
            }
        }
    }

    std::vector<grid_map::GridIndex> removed{{99, 99, 99}};
    ASSERT_TRUE(grid.shiftWindowTo({4.0, -4.0, 3.0}, &removed));

    expectCenter(grid, 4.0, -4.0, 3.0);
    ASSERT_EQ(removed.size(), 27U);
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            for (int z = -1; z <= 1; ++z) {
                const grid_map::GridIndex index{x, y, z};
                EXPECT_EQ(grid.at(index), 0);
                EXPECT_EQ(std::count(removed.begin(), removed.end(), index), 1);
            }
        }
    }
}

TEST(GridMap, SingleAxisShiftReportsExactlyTheSlabLeavingTheWindow) {
    grid_map::GridMap<int> grid;
    auto config = validConfig();
    config.length = {3.0, 3.0, 3.0};
    grid.configure(config);

    std::vector<grid_map::GridIndex> removed;
    ASSERT_TRUE(grid.shiftWindowTo({1.0, 0.0, 0.0}, &removed));

    ASSERT_EQ(removed.size(), 9U);
    for (int y = -1; y <= 1; ++y) {
        for (int z = -1; z <= 1; ++z) {
            EXPECT_EQ(std::count(removed.begin(), removed.end(),
                                 grid_map::GridIndex{-1, y, z}),
                      1);
        }
    }
}

TEST(GridMap, RepeatedRollingDoesNotRestoreDataAfterItLeavesTheWindow) {
    grid_map::GridMap<int> grid;
    grid.configure(validConfig());
    const finenav::common::Position3D tracked_position{1.0, 0.0, 0.0};
    grid.atPosition(tracked_position) = 42;

    for (const double center_x : {1.0, 2.0, 3.0}) {
        ASSERT_TRUE(grid.shiftWindowTo({center_x, 0.0, 0.0}));
        ASSERT_TRUE(grid.isInside(tracked_position));
        EXPECT_EQ(grid.atPosition(tracked_position), 42);
    }

    ASSERT_TRUE(grid.shiftWindowTo({4.0, 0.0, 0.0}));
    EXPECT_FALSE(grid.isInside(tracked_position));

    ASSERT_TRUE(grid.shiftWindowTo({3.0, 0.0, 0.0}));
    ASSERT_TRUE(grid.isInside(tracked_position));
    EXPECT_EQ(grid.atPosition(tracked_position), 0);
}

}  // namespace
