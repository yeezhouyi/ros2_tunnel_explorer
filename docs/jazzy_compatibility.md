# ROS2 Jazzy Compatibility Notes

When adapting the TurtleBot3 Nav2 demo configuration from ROS2 Humble (`nav2_bringup`
`0.x`) to ROS2 Jazzy (`nav2_bringup` `1.x`), several parameter-level changes are
required. These are documented here for maintainers working across distros.

## Plugin Class Names: `/` → `::`

ROS2 Jazzy uses the fully-qualified C++ class name (separated by `::`) for plugin
names in YAML configuration, whereas older examples often use a `/` separator.

| Parameter | Humble (`/` form) | Jazzy (`::` form) |
|-----------|-------------------|-------------------|
| `planner_server.GridBased.plugin` | `nav2_navfn_planner/NavfnPlanner` | `nav2_navfn_planner::NavfnPlanner` |
| `behavior_server.spin.plugin` | `nav2_behaviors/Spin` | `nav2_behaviors::Spin` |
| `behavior_server.backup.plugin` | `nav2_behaviors/BackUp` | `nav2_behaviors::BackUp` |
| `behavior_server.drive_on_heading.plugin` | `nav2_behaviors/DriveOnHeading` | `nav2_behaviors::DriveOnHeading` |
| `behavior_server.wait.plugin` | `nav2_behaviors/Wait` | `nav2_behaviors::Wait` |
| `behavior_server.assisted_teleop.plugin` | `nav2_behaviors/AssistedTeleop` | `nav2_behaviors::AssistedTeleop` |

Using the `/` form in Jazzy causes a lifecycle activation failure:

```
[planner_server]: lifecycle transition "activating" failed
  plugin "nav2_navfn_planner/NavfnPlanner" not found
```

## Missing Server Sections

Jazzy's `nav2_bringup` launches several servers by default that were optional or
absent in Humble. The `nav2_params.yaml` must include a section for each, even
when using default settings, or the lifecycle manager will time out waiting for
these nodes to activate.

The following sections were added:

- **`collision_monitor`** — monitors footprint for imminent collisions and can
  stop the robot. Uses `PolygonStop` with a front bumper polygon and `/scan`
  observations.
- **`docking_server`** — handles dock-detect and docking/undocking with
  `SimpleChargingDock` plugin.
- **`route_server`** — manages sequences of waypoint nodes for
  `NavigateThroughPoses`.
- **`map_saver`** — provides `save_map` service for the `/map_server` node.

## Empty `default_nav_to_pose_bt_xml` Removed

The original configuration had:

```yaml
default_nav_to_pose_bt_xml: ""
```

In Jazzy, an empty string overrides the built-in default behaviour tree path and
causes the bt_navigator to fail with:

```
Behavior tree threw exception: Empty Tree
```

The fix is to **remove the line entirely** so the C++ default is used.

## Navigator Plugin Declarations

Jazzy's `bt_navigator` requires explicit plugin declarations for each navigator
type. The following was added:

```yaml
bt_navigator:
  ros__parameters:
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"
```

Without these, the bt_navigator action server does not respond to
`/navigate_to_pose` goals.

## Summary

These changes are specific to the YAML parameter file only — no source code
changes are needed. The fixes are collected in
`tunnel_explorer_bringup/config/nav2_params.yaml`.
