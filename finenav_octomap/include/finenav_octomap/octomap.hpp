#pragma once

#include <cstddef>
#include <memory>
#include <stdexcept>

#include <octomap/OcTree.h>

#include "finenav_common/geometry_types.hpp"

namespace finenav_octomap {

struct OctoMapConfig {
    double resolution{0.1};
    double occupancy_threshold{0.5};
    double hit_probability{0.7};
    double miss_probability{0.4};
};

/**
 * @brief Global (non-rolling) 3D occupancy map backed by octomap::OcTree.
 *
 * Unlike GridMap, there is no window, no shifting and no local-coordinate
 * interface: every query is a direct world-frame point against the octree.
 * The octree grows as points are inserted, so it suits a fixed global map.
 */
class OctoMap {
public:
    using Position = finenav::common::Position3D;
    using Region = finenav::common::Region3D;

    OctoMap() = default;

    void configure(const OctoMapConfig& config) {
        if (config.resolution <= 0.0) {
            throw std::invalid_argument("OctoMap resolution must be positive");
        }
        tree_ = std::make_unique<octomap::OcTree>(config.resolution);
        tree_->setOccupancyThres(config.occupancy_threshold);
        tree_->setProbHit(config.hit_probability);
        tree_->setProbMiss(config.miss_probability);
        configured_ = true;
    }

    bool isConfigured() const noexcept { return configured_; }

    /// Insert a single point, marking the node occupied (default) or free.
    void insertPoint(const Position& position, bool occupied = true) {
        tree_->updateNode(toPoint3d(position), occupied);
    }

    /// Cast a ray from `origin` to `end`: free space along the ray, occupied at
    /// the endpoint. Typical sensor-model update for a global map.
    void insertRay(const Position& origin, const Position& end) {
        tree_->insertRay(toPoint3d(origin), toPoint3d(end));
    }

    bool isOccupied(const Position& position) const {
        const auto* node = tree_->search(toPoint3d(position));
        return node != nullptr && tree_->isNodeOccupied(node);
    }

    /// Occupancy probability in [0, 1]; 0 when the node has never been updated.
    double getOccupancy(const Position& position) const {
        const auto* node = tree_->search(toPoint3d(position));
        return node != nullptr ? node->getOccupancy() : 0.0;
    }

    Region getBounds() const {
        double min_x = 0.0, min_y = 0.0, min_z = 0.0;
        double max_x = 0.0, max_y = 0.0, max_z = 0.0;
        tree_->getMetricMin(min_x, min_y, min_z);
        tree_->getMetricMax(max_x, max_y, max_z);
        return {{min_x, min_y, min_z}, {max_x, max_y, max_z}};
    }

    double getResolution() const { return tree_->getResolution(); }

    std::size_t getNumLeafNodes() const { return tree_->getNumLeafNodes(); }

    void clear() { tree_->clear(); }

private:
    static octomap::point3d toPoint3d(const Position& position) {
        return {static_cast<float>(position.x),
                static_cast<float>(position.y),
                static_cast<float>(position.z)};
    }

    std::unique_ptr<octomap::OcTree> tree_;
    bool configured_{false};
};

}  // namespace finenav_octomap
