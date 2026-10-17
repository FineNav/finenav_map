# finenav_grid_map

## Overview

`finenav_grid_map` is a header-only C++20 rolling 3D grid container for FineNav
map plugins. It provides `GridIndex`, `GridMapConfig`, and `GridMap<T>` in the
`finenav_grid_map` namespace, using `Position3D` and `Region3D` from
`finenav_common`.

The grid handles world-to-cell conversion, bounds checking, and circular
storage. Moving the window preserves overlapping cells and clears cells that
leave the window. Each consuming plugin defines its own cell data and update
policy.

Consumers declare a `finenav_grid_map` dependency in `package.xml` and use
`find_package(finenav_grid_map REQUIRED)` in CMake.

## Parameters

Pass a `finenav_grid_map::GridMapConfig` to `GridMap<T>::configure()`. These
fields are configured through C++; the package does not declare ROS 2 parameters.
Defaults below come from `GridMapConfig`.

| Field | Type | Default | Unit | Constraints and meaning |
| --- | --- | --- | --- | --- |
| `length` | `Position3D` | `{6.0, 6.0, 6.0}` | m | Requested window lengths along x, y, and z. Every component must be finite and greater than zero. |
| `resolution` | `double` | `0.1` | m/cell | Cell size shared by all three axes. Must be finite and greater than zero. |
| `origin` | `Position3D` | `{0.0, 0.0, 0.0}` | m | World position of logical cell `{0, 0, 0}`, which is also the window center. Every component must be finite. |

`Position3D` is `finenav::common::Position3D`; its components are `x`, `y`,
and `z`. Each axis has `ceil(length / resolution)` cells, increased by one
when that count is even so the window has a central cell. Axis sizes and the
combined cell count must fit the index and storage capacities.

Calling `configure()` allocates fresh storage and resets cell data. Invalid
configuration or allocation failure leaves the previous grid unchanged. Use
`shiftWindowTo()` to move an existing window in whole-cell increments while
retaining overlapping data.

## Complete reference configuration

Save the following as `grid_map_reference.cpp`. It contains every configuration
field with its declared default value and configures a complete grid instance.

```cpp
#include "finenav_grid_map/grid_map.hpp"

int main() {
    finenav_grid_map::GridMapConfig config;
    config.length = {6.0, 6.0, 6.0};  // Requested x/y/z window lengths in metres.
    config.resolution = 0.1;         // Cell size in metres.
    config.origin = {0.0, 0.0, 0.0};  // Initial window center in world coordinates.

    finenav_grid_map::GridMap<int> grid;
    grid.configure(config);
    return 0;
}
```

The `Temperal_voxel_map` plugin constructs its own `GridMapConfig` from its
`map_length` and `resolution` parameters. Configure those values in that
plugin's ROS 2 parameter file when using the voxel map.

## License

This package is licensed under the [Mozilla Public License 2.0 (MPL-2.0)](LICENSE).
