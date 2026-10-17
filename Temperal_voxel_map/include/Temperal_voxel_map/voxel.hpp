#pragma once

#include <chrono>

namespace Temperal_voxel_map {

struct Voxel {
    using Clock = std::chrono::steady_clock;

    bool occupied{false};
    float z_height{0.0F};
    Clock::time_point last_update{};
};

}  // namespace Temperal_voxel_map
