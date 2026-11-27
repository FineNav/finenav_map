#pragma once

#include <cmath>
#include <cstddef>
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "finenav_common/geometry_types.hpp"

namespace finenav_grid_map {

struct GridIndex {
    int x{0};
    int y{0};
    int z{0};

    friend constexpr bool operator==(const GridIndex&, const GridIndex&) = default;
    friend constexpr GridIndex operator+(GridIndex a, GridIndex b) noexcept {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }
    friend constexpr GridIndex operator-(GridIndex a, GridIndex b) noexcept {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }
    friend constexpr GridIndex operator-(GridIndex a) noexcept {
        return {-a.x, -a.y, -a.z};
    }
};

struct GridMapConfig {
    finenav::common::Position3D length{6.0, 6.0, 6.0};
    double resolution{0.1};
    finenav::common::Position3D origin{0.0, 0.0, 0.0};
};

// Inclusive logical-cell bounds. A translated box difference has at most
// three disjoint slabs; no intermediate per-cell index vector is needed.
struct GridIndexRegion {
    GridIndex min;
    GridIndex max;

    template<class Fn>
    void forEachIndex(Fn&& fn) const {
        for (int x = min.x; x <= max.x; ++x)
            for (int y = min.y; y <= max.y; ++y)
                for (int z = min.z; z <= max.z; ++z)
                    fn(GridIndex{x, y, z});
    }
};

struct GridDifferenceRegions {
    std::array<GridIndexRegion, 3> regions{};
    std::size_t count = 0;
    const GridIndexRegion* begin() const { return regions.data(); }
    const GridIndexRegion* end() const { return regions.data() + count; }
};

// A - (A + shift), expressed in A's logical coordinates. size must be positive
// and odd. Check non-overlap before arithmetic, including INT_MIN/MAX jumps.
inline GridDifferenceRegions differenceRegions(GridIndex size, GridIndex shift) {
    const GridIndex lo{-size.x / 2, -size.y / 2, -size.z / 2};
    const GridIndex hi{size.x / 2, size.y / 2, size.z / 2};
    GridDifferenceRegions result;
    if (static_cast<int64_t>(shift.x) >= size.x ||
        static_cast<int64_t>(shift.x) <= -static_cast<int64_t>(size.x) ||
        static_cast<int64_t>(shift.y) >= size.y ||
        static_cast<int64_t>(shift.y) <= -static_cast<int64_t>(size.y) ||
        static_cast<int64_t>(shift.z) >= size.z ||
        static_cast<int64_t>(shift.z) <= -static_cast<int64_t>(size.z)) {
        result.regions[result.count++] = {lo, hi};
        return result;
    }
    GridIndex remaining_lo = lo;
    GridIndex remaining_hi = hi;
    const auto axis = [&](int delta, int& lower, int& upper) {
        if (delta > 0) lower += delta;
        else upper += delta;
    };
    // Restrict subsequent slabs to the overlap on previously handled axes.
    if (shift.x > 0) {
        result.regions[result.count++] = {lo, {lo.x + shift.x - 1, hi.y, hi.z}};
    } else if (shift.x < 0) {
        result.regions[result.count++] = {{hi.x + shift.x + 1, lo.y, lo.z}, hi};
    }
    axis(shift.x, remaining_lo.x, remaining_hi.x);
    if (shift.y > 0) {
        result.regions[result.count++] = {remaining_lo,
            {remaining_hi.x, lo.y + shift.y - 1, hi.z}};
    } else if (shift.y < 0) {
        result.regions[result.count++] = {{remaining_lo.x, hi.y + shift.y + 1, lo.z},
            remaining_hi};
    }
    axis(shift.y, remaining_lo.y, remaining_hi.y);
    if (shift.z > 0) {
        result.regions[result.count++] = {remaining_lo,
            {remaining_hi.x, remaining_hi.y, lo.z + shift.z - 1}};
    } else if (shift.z < 0) {
        result.regions[result.count++] = {{remaining_lo.x, remaining_lo.y, hi.z + shift.z + 1},
            remaining_hi};
    }
    return result;
}

/**
 * @brief Fixed-resolution rolling grid backed by a circular vector.
 *
 * Logical indices are centred around zero. `origin` is the world position of
 * logical index zero, and `start_index` maps logical cells to the circular
 * backing storage. Moving the window keeps overlapping cells in place and
 * resets cells that leave the old/new overlap.
 */
template <typename T>
class GridMap {
public:
    using DataType = T;
    using Position = finenav::common::Position3D;
    using Region = finenav::common::Region3D;

    GridMap() = default;

    void configure(const GridMapConfig& config) {
        if (!isFinite(config.length) || !std::isfinite(config.resolution) ||
            !isFinite(config.origin)) {
            throw std::invalid_argument("GridMap configuration values must be finite");
        }
        if (config.resolution <= 0.0 || config.length.x <= 0.0 ||
            config.length.y <= 0.0 || config.length.z <= 0.0) {
            throw std::invalid_argument("GridMap length and resolution must be positive");
        }

        const GridIndex new_size{
            oddCellCount(config.length.x, config.resolution),
            oddCellCount(config.length.y, config.resolution),
            oddCellCount(config.length.z, config.resolution),
        };
        const std::size_t total_size = checkedCellCount(new_size, data_.max_size());
        std::vector<T> new_data(total_size);

        resolution_ = config.resolution;
        origin_ = config.origin;
        size_ = new_size;
        half_size_ = {new_size.x / 2, new_size.y / 2, new_size.z / 2};
        data_.swap(new_data);
        start_index_ = {0, 0, 0};
        configured_ = true;
    }

    T& at(const GridIndex& index) {
        return data_.at(bufferIndex(index));
    }

    const T& at(const GridIndex& index) const {
        return data_.at(bufferIndex(index));
    }

    T& atPosition(const Position& position) {
        return at(getIndex(position));
    }

    const T& atPosition(const Position& position) const {
        return at(getIndex(position));
    }

    GridIndex getIndex(const Position& position) const {
        if (!isFinite(position)) {
            throw std::invalid_argument("GridMap position must be finite");
        }
        return {
            static_cast<int>(std::lround((position.x - origin_.x) / resolution_)),
            static_cast<int>(std::lround((position.y - origin_.y) / resolution_)),
            static_cast<int>(std::lround((position.z - origin_.z) / resolution_)),
        };
    }

    Position getPosition(const GridIndex& index) const {
        return {
            origin_.x + static_cast<double>(index.x) * resolution_,
            origin_.y + static_cast<double>(index.y) * resolution_,
            origin_.z + static_cast<double>(index.z) * resolution_,
        };
    }

    bool isInside(const GridIndex& index) const noexcept {
        return configured_ && index.x >= -half_size_.x && index.x <= half_size_.x &&
               index.y >= -half_size_.y && index.y <= half_size_.y &&
               index.z >= -half_size_.z && index.z <= half_size_.z;
    }

    bool isInside(const Position& position) const noexcept {
        if (!isFinite(position)) {
            return false;
        }
        return isInside(getIndex(position));
    }

    Region getWindowBounds() const {
        return {getPosition(-half_size_), getPosition(half_size_)};
    }

    Position getWindowCenter() const noexcept { return origin_; }

    GridIndex getMinIndex() const noexcept { return -half_size_; }
    GridIndex getMaxIndex() const noexcept { return half_size_; }
    GridIndex getSize() const noexcept { return size_; }
    double getResolution() const noexcept { return resolution_; }

    /**
     * @brief Shift the world origin by whole voxels.
     * @param position requested new world origin
     * @param removed optional logical indices cleared while leaving the window
     * @return true when at least one voxel shift was applied
     */
    bool shiftWindowTo(const Position& position,
                       std::vector<GridIndex>* removed = nullptr) {
        if (removed != nullptr) removed->clear();
        return shiftWindowToWithCallback(position,
            [removed](const GridIndex& index, const T&) {
                if (removed != nullptr) removed->push_back(index);
            });
    }

    // Invoke outgoing(index, old_value) immediately before clearing each
    // leaving cell. The callback must not mutate/reconfigure this grid.
    // incoming receives disjoint bounds in the NEW logical coordinate frame.
    template<class Fn>
    bool shiftWindowToWithCallback(const Position& position, Fn&& outgoing,
                                  GridDifferenceRegions* incoming = nullptr) {
        if (incoming != nullptr) *incoming = {};
        if (!isFinite(position)) {
            throw std::invalid_argument("GridMap position must be finite");
        }
        if (!configured_) {
            return false;
        }
        const Position rounded{
            std::round((position.x - origin_.x) / resolution_),
            std::round((position.y - origin_.y) / resolution_),
            std::round((position.z - origin_.z) / resolution_)};
        if (!isFinite(rounded) ||
            (rounded.x == 0 && rounded.y == 0 && rounded.z == 0)) return false;
        const Position new_origin{origin_.x + rounded.x * resolution_,
                                  origin_.y + rounded.y * resolution_,
                                  origin_.z + rounded.z * resolution_};
        if (!isFinite(new_origin)) return false;
        // Saturation is only for finding the difference, never for the center.
        const GridIndex shift{
            static_cast<int>(std::clamp(rounded.x, -double(size_.x), double(size_.x))),
            static_cast<int>(std::clamp(rounded.y, -double(size_.y), double(size_.y))),
            static_cast<int>(std::clamp(rounded.z, -double(size_.z), double(size_.z)))};
        for (const auto& region : differenceRegions(size_, shift)) {
            region.forEachIndex([&](const GridIndex& index) {
                auto& cell = at(index);
                outgoing(index, std::as_const(cell));
                cell = T{};
            });
        }
        if (incoming != nullptr) *incoming = differenceRegions(size_, -shift);
        origin_ = new_origin;
        start_index_ = {wrap(int64_t(start_index_.x) + shift.x, size_.x),
                        wrap(int64_t(start_index_.y) + shift.y, size_.y),
                        wrap(int64_t(start_index_.z) + shift.z, size_.z)};
        return true;
    }

private:
    static bool isFinite(const Position& position) noexcept {
        return std::isfinite(position.x) && std::isfinite(position.y) &&
               std::isfinite(position.z);
    }

    static int oddCellCount(double length, double resolution) {
        const double raw_count = std::ceil(length / resolution);
        if (!std::isfinite(raw_count) ||
            raw_count > static_cast<double>(std::numeric_limits<int>::max())) {
            throw std::length_error("GridMap axis exceeds GridIndex capacity");
        }

        int count = static_cast<int>(raw_count);
        if (count % 2 == 0) {
            if (count == std::numeric_limits<int>::max()) {
                throw std::length_error("GridMap odd axis size exceeds GridIndex capacity");
            }
            ++count;
        }
        return count;
    }

    static std::size_t checkedCellCount(const GridIndex& size,
                                        std::size_t max_size) {
        std::size_t total = 1;
        for (const int dimension : {size.x, size.y, size.z}) {
            const auto extent = static_cast<std::size_t>(dimension);
            if (total > max_size / extent) {
                throw std::length_error("GridMap cell count exceeds storage capacity");
            }
            total *= extent;
        }
        return total;
    }

    static int wrap(int64_t value, int size) noexcept {
        const int remainder = value % size;
        return remainder < 0 ? remainder + size : remainder;
    }

    static GridIndex wrap(const GridIndex& index, const GridIndex& size) noexcept {
        return {wrap(index.x, size.x), wrap(index.y, size.y), wrap(index.z, size.z)};
    }

    std::size_t bufferIndex(const GridIndex& index) const {
        if (!isInside(index)) {
            throw std::out_of_range("GridMap index is outside the rolling window");
        }
        const GridIndex wrapped{
            wrap(int64_t(index.x) + half_size_.x + start_index_.x, size_.x),
            wrap(int64_t(index.y) + half_size_.y + start_index_.y, size_.y),
            wrap(int64_t(index.z) + half_size_.z + start_index_.z, size_.z)};
        return static_cast<std::size_t>(wrapped.z) +
               static_cast<std::size_t>(wrapped.y) * static_cast<std::size_t>(size_.z) +
               static_cast<std::size_t>(wrapped.x) * static_cast<std::size_t>(size_.y) *
                   static_cast<std::size_t>(size_.z);
    }

    std::vector<T> data_;
    Position origin_{0.0, 0.0, 0.0};
    GridIndex size_{0, 0, 0};
    GridIndex half_size_{0, 0, 0};
    GridIndex start_index_{0, 0, 0};
    double resolution_{1.0};
    bool configured_{false};
};

}  // namespace finenav_grid_map
