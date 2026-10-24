#pragma once

#include <cmath>
#include <cstddef>
#include <algorithm>
#include <limits>
#include <stdexcept>
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
        return configured_ && std::abs(index.x) <= half_size_.x &&
               std::abs(index.y) <= half_size_.y && std::abs(index.z) <= half_size_.z;
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
     *
     * Clears only the cells in "old window \ new window" and keeps the
     * overlap; cells that leave the window are reset to T{}.
     */
    bool shiftWindowTo(const Position& position,
                       std::vector<GridIndex>* removed = nullptr) {
        if (!isFinite(position)) {
            throw std::invalid_argument("GridMap position must be finite");
        }
        if (!configured_) {
            return false;
        }
        if (removed != nullptr) {
            removed->clear();
        }

        const GridIndex shift{
            static_cast<int>(std::lround((position.x - origin_.x) / resolution_)),
            static_cast<int>(std::lround((position.y - origin_.y) / resolution_)),
            static_cast<int>(std::lround((position.z - origin_.z) / resolution_)),
        };
        if (shift == GridIndex{}) {
            return false;
        }

        // Old index `old` maps to new index `old - shift`; the new window in
        // old coordinates is [-half + shift, half + shift].
        std::vector<GridIndex> leaving;
        getDifferenceSet(shift, leaving);

        for (const GridIndex& old_index : leaving) {
            if (removed != nullptr) {
                removed->push_back(old_index);
            }
            at(old_index) = T{};
        }

        origin_.x += static_cast<double>(shift.x) * resolution_;
        origin_.y += static_cast<double>(shift.y) * resolution_;
        origin_.z += static_cast<double>(shift.z) * resolution_;
        start_index_ = wrap(start_index_ + shift, size_);
        return true;
    }

    /**
     * @brief Copy the z column at logical (x, y) into `result`.
     * @param x logical x index
     * @param y logical y index
     * @param result output buffer, cleared and filled with size_.z values
     * @throw std::out_of_range when (x, y) is outside the window
     */
    void getVoxelsAlongZ(const int& x, const int& y, std::vector<T>& result) const {
        if (!configured_ || x < -half_size_.x || x > half_size_.x ||
            y < -half_size_.y || y > half_size_.y) {
            throw std::out_of_range("GridMap (x, y) is outside the rolling window");
        }

        result.clear();
        if (result.capacity() < static_cast<std::size_t>(size_.z)) {
            result.reserve(static_cast<std::size_t>(size_.z));
        }

        // x/y are constant along the column: resolve their offsets once.
        const int wrapped_x = wrap(x + half_size_.x + start_index_.x, size_.x);
        const int wrapped_y = wrap(y + half_size_.y + start_index_.y, size_.y);
        // Use std::size_t to avoid int overflow for large windows.
        const std::size_t base_offset =
            static_cast<std::size_t>(wrapped_y) * static_cast<std::size_t>(size_.z) +
            static_cast<std::size_t>(wrapped_x) * static_cast<std::size_t>(size_.y) *
                static_cast<std::size_t>(size_.z);

        for (int z = -half_size_.z; z <= half_size_.z; ++z) {
            const int wrapped_z = wrap(z + half_size_.z + start_index_.z, size_.z);
            result.push_back(data_[base_offset + static_cast<std::size_t>(wrapped_z)]);
        }
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

    static int wrap(int value, int size) noexcept {
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
        const GridIndex unwrapped = index + half_size_ + start_index_;
        const GridIndex wrapped = wrap(unwrapped, size_);
        return static_cast<std::size_t>(wrapped.z) +
               static_cast<std::size_t>(wrapped.y) * static_cast<std::size_t>(size_.z) +
               static_cast<std::size_t>(wrapped.x) * static_cast<std::size_t>(size_.y) *
                   static_cast<std::size_t>(size_.z);
    }

    /**
     * @brief Compute "old window \ new window" in old logical indices.
     *
     * A = [-half, half], B = [-half + shift, half + shift]. Enumerates A \ B
     * by z -> x -> y layers; returns all of A when B lies outside A.
     */
    void getDifferenceSet(const GridIndex& shift, std::vector<GridIndex>& difference) const {
        difference.clear();

        const GridIndex a_min = -half_size_;
        const GridIndex a_max = half_size_;
        const GridIndex b_min = -half_size_ + shift;
        const GridIndex b_max = half_size_ + shift;

        auto add_range = [&](int x_start, int x_end, int y_start, int y_end,
                             int z_start, int z_end) {
            x_start = std::max(x_start, a_min.x);
            x_end = std::min(x_end, a_max.x);
            y_start = std::max(y_start, a_min.y);
            y_end = std::min(y_end, a_max.y);
            z_start = std::max(z_start, a_min.z);
            z_end = std::min(z_end, a_max.z);

            if (x_start > x_end || y_start > y_end || z_start > z_end) {
                return;
            }
            for (int x = x_start; x <= x_end; ++x) {
                for (int y = y_start; y <= y_end; ++y) {
                    for (int z = z_start; z <= z_end; ++z) {
                        difference.push_back(GridIndex{x, y, z});
                    }
                }
            }
        };

        // B lies entirely outside A.
        if (b_min.x > a_max.x || b_max.x < a_min.x ||
            b_min.y > a_max.y || b_max.y < a_min.y ||
            b_min.z > a_max.z || b_max.z < a_min.z) {
            add_range(a_min.x, a_max.x, a_min.y, a_max.y, a_min.z, a_max.z);
            return;
        }

        if (a_min.z <= b_max.z && b_max.z <= a_max.z) {
            add_range(a_min.x, a_max.x, a_min.y, a_max.y, b_max.z + 1, a_max.z);

            if (a_min.x <= b_min.x && b_min.x <= a_max.x) {
                add_range(a_min.x, b_min.x - 1, a_min.y, a_max.y, a_min.z, b_max.z);

                if (a_min.y <= b_min.y && b_min.y <= a_max.y) {
                    add_range(b_min.x, b_max.x, a_min.y, b_min.y - 1, a_min.z, b_max.z);
                } else if (a_min.y <= b_max.y && b_max.y <= a_max.y) {
                    add_range(b_min.x, b_max.x, b_max.y + 1, a_max.y, a_min.z, b_max.z);
                }
            } else if (a_min.x <= b_max.x && b_max.x <= a_max.x) {
                add_range(b_max.x + 1, a_max.x, a_min.y, a_max.y, a_min.z, b_max.z);

                if (a_min.y <= b_min.y && b_min.y <= a_max.y) {
                    add_range(b_min.x, b_max.x, a_min.y, b_min.y - 1, a_min.z, b_max.z);
                } else if (a_min.y <= b_max.y && b_max.y <= a_max.y) {
                    add_range(b_min.x, b_max.x, b_max.y + 1, a_max.y, a_min.z, b_max.z);
                }
            }
        } else if (a_min.z <= b_min.z && b_min.z <= a_max.z) {
            add_range(a_min.x, a_max.x, a_min.y, a_max.y, a_min.z, b_min.z - 1);

            if (a_min.x <= b_min.x && b_min.x <= a_max.x) {
                add_range(a_min.x, b_min.x - 1, a_min.y, a_max.y, b_min.z, a_max.z);

                if (a_min.y <= b_min.y && b_min.y <= a_max.y) {
                    add_range(b_min.x, b_max.x, a_min.y, b_min.y - 1, b_min.z, a_max.z);
                } else if (a_min.y <= b_max.y && b_max.y <= a_max.y) {
                    add_range(b_min.x, b_max.x, b_max.y + 1, a_max.y, b_min.z, a_max.z);
                }
            } else if (a_min.x <= b_max.x && b_max.x <= a_max.x) {
                add_range(b_max.x + 1, a_max.x, a_min.y, a_max.y, b_min.z, a_max.z);

                if (a_min.y <= b_min.y && b_min.y <= a_max.y) {
                    add_range(b_min.x, b_max.x, a_min.y, b_min.y - 1, b_min.z, a_max.z);
                } else if (a_min.y <= b_max.y && b_max.y <= a_max.y) {
                    add_range(b_min.x, b_max.x, b_max.y + 1, a_max.y, b_min.z, a_max.z);
                }
            }
        }
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
