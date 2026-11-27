
#include <map>
#include <set>
#include <tuple>
#include <random>
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

#include "finenav_grid_map/grid_map.hpp"

namespace {

finenav_grid_map::GridMapConfig validConfig() {
    finenav_grid_map::GridMapConfig config;
    config.length = {5.0, 5.0, 5.0};
    config.resolution = 1.0;
    config.origin = {0.0, 0.0, 0.0};
    return config;
}

void expectCenter(const finenav_grid_map::GridMap<int>& grid, double x, double y, double z) {
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
            finenav_grid_map::GridMap<int> grid;
            auto config = validConfig();
            config.resolution = invalid;
            EXPECT_THROW(grid.configure(config), std::invalid_argument);
        }
        {
            finenav_grid_map::GridMap<int> grid;
            auto config = validConfig();
            config.length.x = invalid;
            EXPECT_THROW(grid.configure(config), std::invalid_argument);
        }
        {
            finenav_grid_map::GridMap<int> grid;
            auto config = validConfig();
            config.origin.z = invalid;
            EXPECT_THROW(grid.configure(config), std::invalid_argument);
        }
    }
}

TEST(GridMap, RejectsNonPositiveLengthAndResolution) {
    for (const double invalid : {0.0, -1.0}) {
        {
            finenav_grid_map::GridMap<int> grid;
            auto config = validConfig();
            config.resolution = invalid;
            EXPECT_THROW(grid.configure(config), std::invalid_argument);
        }
        {
            finenav_grid_map::GridMap<int> grid;
            auto config = validConfig();
            config.length.y = invalid;
            EXPECT_THROW(grid.configure(config), std::invalid_argument);
        }
    }
}

TEST(GridMap, RejectsAxisCellCountOverflowWithoutChangingExistingMap) {
    finenav_grid_map::GridMap<int> grid;
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
    finenav_grid_map::GridMap<int> grid;
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
    finenav_grid_map::GridMap<int> grid;
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
    finenav_grid_map::GridMap<int> grid;
    finenav_grid_map::GridMapConfig config;
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
    finenav_grid_map::GridMap<int> grid;
    finenav_grid_map::GridMapConfig config;
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
    finenav_grid_map::GridMap<int> grid;
    auto config = validConfig();
    config.length = {3.0, 3.0, 3.0};
    grid.configure(config);
    grid.at({-1, -1, -1}) = 11;
    grid.at({0, 0, 0}) = 22;
    grid.at({1, 1, 1}) = 33;

    std::vector<finenav_grid_map::GridIndex> removed;
    ASSERT_TRUE(grid.shiftWindowTo({1.0, 1.0, 1.0}, &removed));

    expectCenter(grid, 1.0, 1.0, 1.0);
    EXPECT_EQ(grid.at({-1, -1, -1}), 22);
    EXPECT_EQ(grid.at({0, 0, 0}), 33);
    EXPECT_EQ(grid.at({1, 1, 1}), 0);
    EXPECT_EQ(removed.size(), 19U);
}

TEST(GridMap, ShiftBeyondTheWindowClearsEveryCellAndReportsEveryOldIndex) {
    finenav_grid_map::GridMap<int> grid;
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

    std::vector<finenav_grid_map::GridIndex> removed{{99, 99, 99}};
    ASSERT_TRUE(grid.shiftWindowTo({4.0, -4.0, 3.0}, &removed));

    expectCenter(grid, 4.0, -4.0, 3.0);
    ASSERT_EQ(removed.size(), 27U);
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            for (int z = -1; z <= 1; ++z) {
                const finenav_grid_map::GridIndex index{x, y, z};
                EXPECT_EQ(grid.at(index), 0);
                EXPECT_EQ(std::count(removed.begin(), removed.end(), index), 1);
            }
        }
    }
}

TEST(GridMap, SingleAxisShiftReportsExactlyTheSlabLeavingTheWindow) {
    finenav_grid_map::GridMap<int> grid;
    auto config = validConfig();
    config.length = {3.0, 3.0, 3.0};
    grid.configure(config);

    std::vector<finenav_grid_map::GridIndex> removed;
    ASSERT_TRUE(grid.shiftWindowTo({1.0, 0.0, 0.0}, &removed));

    ASSERT_EQ(removed.size(), 9U);
    for (int y = -1; y <= 1; ++y) {
        for (int z = -1; z <= 1; ++z) {
            EXPECT_EQ(std::count(removed.begin(), removed.end(),
                                 finenav_grid_map::GridIndex{-1, y, z}),
                      1);
        }
    }
}

TEST(GridMap, RepeatedRollingDoesNotRestoreDataAfterItLeavesTheWindow) {
    finenav_grid_map::GridMap<int> grid;
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


using IndexTuple = std::tuple<int, int, int>;
IndexTuple tupleOf(finenav_grid_map::GridIndex i) { return {i.x, i.y, i.z}; }

TEST(GridDifference, MatchesBruteForceForAllDirectionsAndDegenerateAxes) {
    using namespace finenav_grid_map;
    for (GridIndex size : {GridIndex{1, 1, 1}, GridIndex{1, 3, 5}, GridIndex{5, 3, 7}}) {
        GridIndexRegion all{{-size.x / 2, -size.y / 2, -size.z / 2},
                            {size.x / 2, size.y / 2, size.z / 2}};
        for (int dx = -size.x - 1; dx <= size.x + 1; ++dx)
        for (int dy = -size.y - 1; dy <= size.y + 1; ++dy)
        for (int dz = -size.z - 1; dz <= size.z + 1; ++dz) {
            const GridIndex shift{dx, dy, dz};
            std::set<IndexTuple> expected, actual;
            all.forEachIndex([&](GridIndex i) {
                auto j = i - shift;
                if (std::abs(j.x) > size.x / 2 || std::abs(j.y) > size.y / 2 ||
                    std::abs(j.z) > size.z / 2) expected.insert(tupleOf(i));
            });
            for (const auto& box : differenceRegions(size, shift)) {
                box.forEachIndex([&](GridIndex i) {
                    ASSERT_TRUE(actual.insert(tupleOf(i)).second) << "duplicate edge/corner";
                });
            }
            ASSERT_EQ(actual, expected) << dx << ',' << dy << ',' << dz;
        }
    }
}

TEST(GridDifference, ExtremeIntegerShiftsReturnWholeBoxWithoutOverflow) {
    using namespace finenav_grid_map;
    for (int displacement : {std::numeric_limits<int>::min(), std::numeric_limits<int>::max()}) {
        size_t count = 0;
        for (const auto& box : differenceRegions({3, 5, 7}, {0, displacement, 0}))
            box.forEachIndex([&](GridIndex) { ++count; });
        EXPECT_EQ(count, 105u);
    }
}

TEST(GridMap, RandomWalkMatchesWorldCellOracleAndKeepsOverlapAddresses) {
    using namespace finenav_grid_map;
    GridMap<int> grid;
    auto config = validConfig();
    config.length = {5, 3, 7};
    grid.configure(config);
    const GridIndexRegion all{grid.getMinIndex(), grid.getMaxIndex()};
    std::mt19937 random(12597);
    std::uniform_int_distribution<int> step(-8, 8);
    int next_value = 1;
    for (int iteration = 0; iteration < 150; ++iteration) {
        std::map<IndexTuple, std::pair<int, const int*>> old;
        all.forEachIndex([&](GridIndex i) {
            grid.at(i) = next_value++;
            auto p = grid.getPosition(i);
            old[{int(p.x), int(p.y), int(p.z)}] = {grid.at(i), &grid.at(i)};
        });
        auto center = grid.getWindowCenter();
        const GridIndex shift{step(random), step(random), step(random)};
        std::set<IndexTuple> removed;
        GridDifferenceRegions incoming;
        const bool moved = grid.shiftWindowToWithCallback(
            {center.x + shift.x, center.y + shift.y, center.z + shift.z},
            [&](GridIndex i, const int& value) {
                ASSERT_EQ(value, grid.at(i));
                ASSERT_TRUE(removed.insert(tupleOf(i)).second);
            }, &incoming);
        ASSERT_EQ(moved, shift != GridIndex{});
        size_t entered = 0;
        all.forEachIndex([&](GridIndex i) {
            auto p = grid.getPosition(i);
            auto found = old.find({int(p.x), int(p.y), int(p.z)});
            if (found == old.end()) { EXPECT_EQ(grid.at(i), 0); ++entered; }
            else {
                EXPECT_EQ(grid.at(i), found->second.first);
                EXPECT_EQ(&grid.at(i), found->second.second);
            }
            auto after = i - shift;
            EXPECT_EQ(removed.count(tupleOf(i)), grid.isInside(after) ? 0u : 1u);
        });
        std::set<IndexTuple> new_indices;
        for (const auto& box : incoming) box.forEachIndex([&](GridIndex i) {
            EXPECT_TRUE(new_indices.insert(tupleOf(i)).second);
            auto p = grid.getPosition(i);
            EXPECT_EQ(old.count({int(p.x), int(p.y), int(p.z)}), 0u);
        });
        EXPECT_EQ(new_indices.size(), entered);
        EXPECT_EQ(removed.size(), entered);
    }
}

TEST(GridMap, HugeFiniteShiftStaysAlignedAndClearsOnce) {
    using namespace finenav_grid_map;
    GridMap<int> grid;
    auto config = validConfig();
    config.origin = {0.25, 0.25, 0.25};
    config.length = {1, 1, 1};
    config.resolution = 0.5;
    grid.configure(config);
    grid.at({0, 0, 0}) = 9;
    std::vector<GridIndex> removed;
    ASSERT_TRUE(grid.shiftWindowTo({double(std::numeric_limits<int>::max()) + 100.4, 0.25, 0.25}, &removed));
    EXPECT_EQ(removed.size(), 27u);
    EXPECT_EQ(grid.at({0, 0, 0}), 0);
    EXPECT_DOUBLE_EQ(std::remainder(grid.getWindowCenter().x - 0.25, 0.5), 0.0);
    removed.push_back({9, 9, 9});
    EXPECT_FALSE(grid.shiftWindowTo(grid.getWindowCenter(), &removed));
    EXPECT_TRUE(removed.empty());
}


TEST(GridMap, HalfCellRequestsRoundAwayFromZeroAndDoNotAccumulateSubCellDrift) {
    finenav_grid_map::GridMap<int> grid;
    grid.configure(validConfig());
    size_t callbacks = 0;
    EXPECT_FALSE(grid.shiftWindowToWithCallback({0.49,-0.49,0},
        [&](const auto&,int){++callbacks;}));
    EXPECT_EQ(callbacks,0u);
    ASSERT_TRUE(grid.shiftWindowTo({0.5,-0.5,1.5}));
    expectCenter(grid,1,-1,2);
    EXPECT_FALSE(grid.shiftWindowTo({1.49,-1.49,2.49}));
    expectCenter(grid,1,-1,2);
}

TEST(GridMap, UnrepresentableDisplacementDoesNotClearOrMoveWindow) {
    finenav_grid_map::GridMap<int> grid;
    auto config=validConfig();
    config.origin.x=-std::numeric_limits<double>::max();
    grid.configure(config);
    grid.at({0,0,0})=17;
    size_t callbacks=0;
    finenav_grid_map::GridDifferenceRegions incoming;
    EXPECT_FALSE(grid.shiftWindowToWithCallback({std::numeric_limits<double>::max(),0,0},
        [&](const auto&,int){++callbacks;}, &incoming));
    EXPECT_EQ(callbacks,0u);
    EXPECT_EQ(incoming.count,0u);
    EXPECT_EQ(grid.at({0,0,0}),17);
    EXPECT_DOUBLE_EQ(grid.getWindowCenter().x,config.origin.x);
}

}  // namespace
