# Temperal_voxel_map

## Overview

`Temperal_voxel_map` is a FineNav `Map` plugin for a rolling three-dimensional
voxel window, backed by the reusable `finenav_grid_map` container. It buffers
`sensor_msgs/msg/PointCloud2` observations, marks observed voxels as occupied,
and clears them after their configured lifetime without a refresh. Cells that
leave the rolling window are cleared as the window follows the robot.

Clouds in `map_frame` are inserted directly. Clouds in `input_frame` are
transformed using the `RobotState.pose` at the observation timestamp; all other
frame IDs are dropped. Supply points in the robot body frame before publishing
when using `input_frame`; this plugin does not configure sensor extrinsics.
A robot state must be available at each observation timestamp, including for
clouds already in `map_frame`.

## Parameters

The ten plugin parameters below are declared on the MapServer node with the
`Temperal_voxel_map.` prefix. For example, the full ROS parameter name for the
voxel lifetime is `Temperal_voxel_map.decay_time_sec`. All are read-only after
startup except `decay_time_sec`, whose updates take effect on the next map update
cycle. Voxel lifetime uses elapsed steady-clock time since fusion; observation
buffer limits use message timestamps and the server's ROS clock.

| Parameter | Type | Default | Range / unit | Updates | Meaning |
| --- | --- | --- | --- | --- | --- |
| `cloud_topic` | `string` | `/points` | ROS topic name | Startup only | Point cloud subscription, using sensor-data QoS. |
| `input_frame` | `string` | `base_link` | Frame ID | Startup only | Robot body frame transformed into `map_frame` using `RobotState.pose`. |
| `map_frame` | `string` | `map` | Frame ID | Startup only | World frame for the voxel map; matching clouds need no coordinate transform. |
| `map_length` | `double` | `20.0` | Finite, `> 0`, m | Startup only | Requested side length of the cubic window; each axis is rounded up to an odd cell count. |
| `resolution` | `double` | `0.2` | Finite, `> 0`, m/cell | Startup only | Voxel edge length. |
| `decay_time_sec` | `double` | `5.0` | `>= 0`, s | Dynamic | Clear occupied cells older than this lifetime. `0` disables age-based clearing; rolling-window clearing still applies. |
| `max_buffer_size` | `int` | `32` | `>= 0`, messages | Startup only | Maximum pending observations; a full buffer drops its oldest message. `0` removes the count limit. |
| `observation_keep_time_sec` | `double` | `1.0` | `>= 0`, s | Startup only | Drop pending messages older than this before fusion. `0` disables age-based buffer eviction. |
| `source_timeout_sec` | `double` | `0.0` | `>= 0`, s | Startup only | Clear pending observations when the last received message timestamp exceeds this age. `0` disables the timeout. |
| `expected_update_rate_hz` | `double` | `0.0` | `>= 0`, Hz | Startup only | Warn after the gap since the last received message timestamp exceeds `1 / rate`. `0` disables this warning. |

The following parameters belong to FineNav's MapServer, rather than the plugin.
They have no `Temperal_voxel_map.` prefix and are read once when the server is
constructed; changing their ROS values later does not update the active timers
or shift threshold.

| Parameter | Type | Default | Range / unit | Meaning |
| --- | --- | --- | --- | --- |
| `update_rate_hz` | `double` | `10.0` | `> 0`, Hz | Drain observation buffers and run the map update cycle. |
| `shift_rate_hz` | `double` | `50.0` | `> 0`, Hz | Check whether the rolling window must follow the robot. |
| `shift_margin_ratio` | `double` | `0.5` | `[0, 1]`, dimensionless | Shift when any axis's distance from the window centre, divided by that axis's half extent, exceeds this ratio. `0` triggers after leaving the centre; `1` places the threshold at the edge. |

## Complete reference configuration

[config/Temperal_voxel_map.yaml](config/Temperal_voxel_map.yaml) is the
`generate_parameter_library` build-time schema, containing types, defaults, and
validation rules. The following is a ROS 2 runtime parameter file, with all ten
plugin values and the three MapServer defaults:

```yaml
/**/Temperal_voxel_map:
  ros__parameters:
    # FineNav MapServer parameters; no plugin prefix.
    update_rate_hz: 10.0
    shift_rate_hz: 50.0
    shift_margin_ratio: 0.5

    # Temperal_voxel_map plugin parameters.
    Temperal_voxel_map:
      cloud_topic: "/points"
      input_frame: "base_link"
      map_frame: "map"
      map_length: 20.0
      resolution: 0.2
      decay_time_sec: 5.0
      max_buffer_size: 32
      observation_keep_time_sec: 1.0
      source_timeout_sec: 0.0
      expected_update_rate_hz: 0.0
```

Save this block as a runtime YAML file and pass
`--ros-args --params-file /path/to/temperal_voxel_map.runtime.yaml` when starting
the hosting ROS 2 process. The node selector `/**/Temperal_voxel_map` matches a
MapServer named `Temperal_voxel_map` in any namespace, which is the name used by
the current Engine when assembling a WorldView that requires this registered
map. For a differently named MapServer, replace the selector's final component
with that server's actual node name, or use its full node path from
`ros2 node list`. Keep the inner `Temperal_voxel_map` parameter prefix unchanged.
This file configures an instantiated server; the hosting application must load
the registered map and assemble its server separately.

## License

This package is licensed under the [Mozilla Public License 2.0 (MPL-2.0)](LICENSE).
