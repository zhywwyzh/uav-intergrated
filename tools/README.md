# Tools

## Trigger scripts for `scripts/mix.sh`

- `trigger_ego.py`
  - Publish mode trigger `Int32(1)` to `/uav_planner/trigger`, enable ego planning service, then publish one `GoalSet` to `/goal_with_id_from_station`.
  - Example:
    - `python3 tools/trigger_ego.py --x 2.0 --y 1.0 --z 1.2 --yaw 0.0`

- `trigger_track.py`
  - Publish mode trigger `Int32(2)` to `/uav_planner/trigger`, then publish `/tracker_trigger` (default one-shot trigger).
  - By default also publishes fake target odom (`nav_msgs/Odometry`) + ego odom.
  - Example:
    - `python3 tools/trigger_track.py`
  - Trigger only (no fake input):
    - `python3 tools/trigger_track.py --no-fake-inputs`

- `trigger_track_land.py`
  - Publish `/track_land_trigger` directly (track landing command).
  - Example:
    - `python3 tools/trigger_track_land.py`

- `trigger_perch.py`
  - Publish `/perch_trigger` directly (default one-shot trigger).
  - Example:
    - `python3 tools/trigger_perch.py`

## Stop all runtimes

- `scripts/stop_all.sh`
  - Stop all runtimes started by `scripts/mix.sh`.
  - Example:
    - `bash scripts/stop_all.sh`

## Unified mode trigger

- `ego_replan_fsm` subscribes `/uav_planner/trigger` (`std_msgs/Int32`).
- `1` switches to ego planning logic; `2` switches to tracker planning logic.

## Interactive trigger menu

- `scripts/task_pub.sh`
  - Interactive wrapper that calls the new direct trigger tools.
  - Example:
    - `bash scripts/task_pub.sh`
