# finenav_grid_map

Header-only C++20 rolling grid container shared by FineNav map plugins.
It depends on `finenav_common` for `Position3D` and `Region3D`.

The public types are `finenav_grid_map::GridIndex`, `GridMapConfig`, and
`GridMap<T>`. The container handles coordinate conversion, bounds checking,
and circular storage while retaining overlapping cells when the window moves.
Cell data and update policies belong to each consuming plugin.

```cpp
#include "finenav_grid_map/grid_map.hpp"

finenav_grid_map::GridMap<int> grid;
finenav_grid_map::GridMapConfig config;
config.length = {5.0, 5.0, 5.0};
config.resolution = 1.0;
grid.configure(config);

grid.atPosition({1.0, 0.0, 0.0}) = 7;
grid.shiftWindowTo({2.0, 0.0, 0.0});
```

Declare a `finenav_grid_map` dependency in `package.xml` and find it with
`find_package(finenav_grid_map REQUIRED)` in CMake.
