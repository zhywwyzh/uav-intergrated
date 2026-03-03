# uav-planner Workspace

This workspace integrates `ego-planner` and `Elastic-Tracker` into one catkin workspace source tree.

## Layout

- `src/` contains symlinks to selected packages from both projects.
- Duplicate package names are avoided by:
  - Reusing ego's `quadrotor_msgs` for both sides.
  - Renaming Elastic-Tracker package `traj_opt` to `tracker_traj_opt`.

## Included package groups

- Ego side:
  - `quadrotor_msgs`, `uav_utils`, `perception_utils`
  - `traj_utils`, `plan_env`, `path_searching`, `traj_opt`, `plan_manage`, `map_interface`, `mission_fsm`
- Tracker side:
  - `catkin_simple`, `decomp_ros_msgs`, `decomp_ros_utils`
  - `object_detection_msgs`, `target_ekf`, `mapping`, `tracker_traj_opt`, `planning`

## Build

```bash
cd /home/zhywwyzh/workspace/uav-integrated/uav-planner
source /opt/ros/noetic/setup.bash
catkin_make
```

## Trigger mode

- Unified mode topic: `/uav_planner/trigger` (`std_msgs/Int32`)
- `1`: ego planner FSM
- `2`: tracker planner FSM
