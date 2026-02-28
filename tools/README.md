# Tools

## Trigger scripts for `scripts/mix.sh`

- `trigger_ego.py`
  - Enable ego planning service, then publish one `GoalSet` to `/goal_with_id_from_station`.
  - Example:
    - `python3 tools/trigger_ego.py --x 2.0 --y 1.0 --z 1.2 --yaw 0.0`

- `trigger_track.py`
  - Publish `/track_trigger` directly.
  - By default also publishes fake YOLO + odom to satisfy tracker inputs.
  - Example:
    - `python3 tools/trigger_track.py`
  - Trigger only (no fake input):
    - `python3 tools/trigger_track.py --no-fake-inputs`

- `trigger_track_land.py`
  - Publish `/track_land_trigger` directly (track landing command).
  - Example:
    - `python3 tools/trigger_track_land.py`

- `trigger_perch.py`
  - Publish `/perch_trigger` directly.
  - Example:
    - `python3 tools/trigger_perch.py`

## Stop all runtimes

- `scripts/stop_all.sh`
  - Stop all runtimes started by `scripts/mix.sh`.
  - Example:
    - `bash scripts/stop_all.sh`

## Interactive trigger menu

- `scripts/task_pub.sh`
  - Interactive wrapper that calls the new direct trigger tools.
  - Example:
    - `bash scripts/task_pub.sh`
