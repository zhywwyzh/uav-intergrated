# Tools

## Trigger scripts for `scripts/mix.sh`

- `trigger_ego.py`
  - Enable ego planning service, then publish one `GoalSet` trigger to `/ego_trigger`.
  - Example:
    - `python3 tools/trigger_ego.py --x 2.0 --y 1.0 --z 1.2 --yaw 0.0`

- `trigger_track.py`
  - Publish `/track_trigger` directly (default one-shot trigger).
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

## Trigger preemption

- `scripts/mix.sh` now starts `utils/task_trigger_arbiter.py`.
- Any new trigger (`/ego_trigger` or `/goal_with_id_from_station`, `/track_trigger`, `/perch_trigger`) will preempt current tasks first, then execute the latest trigger.

## Interactive trigger menu

- `scripts/task_pub.sh`
  - Interactive wrapper that calls the new direct trigger tools.
  - Example:
    - `bash scripts/task_pub.sh`
