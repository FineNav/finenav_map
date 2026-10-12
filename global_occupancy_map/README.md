# global_occupancy_map

`global_occupancy_map` is a persistent FineNav 3D occupancy Map plugin. It
subscribes to `sensor_msgs/msg/PointCloud2` through `MapRuntime`, transforms
`input_frame` clouds with `RobotState.pose`, and inserts every valid world-frame
endpoint as occupied.

The input contract matches `stvl_map`:

- A cloud in `map_frame` (default `map`) is inserted directly.
- A cloud in `input_frame` (default `base_link`) is transformed with the
  FineNav robot state.
- Other frame IDs are dropped. No sensor-to-base extrinsic is configured.

The map has no rolling window, temporal decay, deletion, or ray-based
free-space update. Occupied endpoints accumulate for the lifetime of the map
server.

Load the plugin before creating its map server:

```cpp
engine->loadAlgoPlugins({"global_occupancy_map/lib"});
engine->createMapServer(
    "global_occupancy_map", "global_occupancy_map_server");
```

The generated parameter namespace is `global_occupancy_map`; see
`config/global_occupancy_map.yaml`. All parameters are read-only after startup.
