#!/bin/bash
# Main control script:
# 1) start ego mapping + track + perch runtimes together
# 2) keep them alive
#
# NOTE:
# - This script no longer depends on a global task dispatcher.
# - Module triggering is now done directly via module-specific topics/tools.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
UAV_WS="$PROJECT_ROOT/uav-planner"
UAV_DEVEL_SPACE="${UAV_DEVEL_SPACE:-devel}"
TMP_DIR="$PROJECT_ROOT/tmp"
mkdir -p "$TMP_DIR"
MIX_PID_FILE="$TMP_DIR/mix.pid"
echo $$ > "$MIX_PID_FILE"

USE_LEGACY_ARBITER="${USE_LEGACY_ARBITER:-0}"

echo "=== Main Control Script Starting (Unified mode-trigger) ==="
echo ""

# PID file paths
EGO_RUN_PID_FILE="$TMP_DIR/run_in_sim.pid"
TRACK_PID_FILE="$TMP_DIR/track_real.pid"
PERCHING_PID_FILE="$TMP_DIR/perching.pid"
ARBITER_PID_FILE="$TMP_DIR/task_trigger_arbiter.pid"

START_PERCH_PID_FILE="$TMP_DIR/start_perch.pid"

EGO_PLANNING_SERVICE="/drone_0_ego_planner_node/planning/enable"

cleanup() {
    echo ""
    echo "=== Received interrupt signal, starting cleanup... ==="

    if [ -f "$START_PERCH_PID_FILE" ]; then
        START_PERCH_PID=$(cat "$START_PERCH_PID_FILE" 2>/dev/null)
        if [ ! -z "$START_PERCH_PID" ] && kill -0 "$START_PERCH_PID" 2>/dev/null; then
            echo "  Terminating start_perch.sh process (PID: $START_PERCH_PID)"
            kill -TERM "$START_PERCH_PID" 2>/dev/null
            sleep 0.2
            kill -KILL "$START_PERCH_PID" 2>/dev/null
        fi
        rm -f "$START_PERCH_PID_FILE"
    fi

    if [ -f "$ARBITER_PID_FILE" ]; then
        ARBITER_PID=$(cat "$ARBITER_PID_FILE" 2>/dev/null)
        if [ ! -z "$ARBITER_PID" ] && kill -0 "$ARBITER_PID" 2>/dev/null; then
            echo "  Terminating task_trigger_arbiter.py (PID: $ARBITER_PID)"
            kill -TERM "$ARBITER_PID" 2>/dev/null
            sleep 0.2
            kill -KILL "$ARBITER_PID" 2>/dev/null
        fi
        rm -f "$ARBITER_PID_FILE"
    fi

    # fallback kill by pattern
    pkill -f "mission_fsm multidrone_sim.launch" 2>/dev/null
    pkill -f "mission_fsm run_in_sim.xml" 2>/dev/null
    pkill -f "planning real_external.launch" 2>/dev/null
    pkill -f "perching.launch" 2>/dev/null
    pkill -f "task_trigger_arbiter.py" 2>/dev/null

    rm -f \
        "$MIX_PID_FILE" \
        "$EGO_RUN_PID_FILE" \
        "$TRACK_PID_FILE" \
        "$PERCHING_PID_FILE" \
        "$ARBITER_PID_FILE"

    echo "=== Cleanup completed, script exiting ==="
    exit 0
}

trap cleanup SIGINT SIGTERM SIGQUIT

set_ego_planning_state() {
    local target_state="$1"
    local retry=0

    while [ $retry -lt 20 ]; do
        if rosservice call "$EGO_PLANNING_SERVICE" "data: $target_state" >/dev/null 2>&1; then
            echo "  Planning state changed: $target_state"
            return 0
        fi
        retry=$((retry + 1))
        sleep 0.2
    done

    echo "  Warning: failed to call planning switch service ($EGO_PLANNING_SERVICE)"
    return 1
}

source_uav_ws() {
    if [ ! -f "$UAV_WS/$UAV_DEVEL_SPACE/setup.sh" ]; then
        echo "  Error: missing $UAV_WS/$UAV_DEVEL_SPACE/setup.sh. Build uav-planner first."
        return 1
    fi
    # shellcheck disable=SC1090
    source "$UAV_WS/$UAV_DEVEL_SPACE/setup.sh"
}

start_ego_mapping() {
    echo "Starting uav-planner ego mapping (planning disabled)..."

    local run_pid=""
    if [ -f "$EGO_RUN_PID_FILE" ]; then
        run_pid=$(cat "$EGO_RUN_PID_FILE" 2>/dev/null)
    fi
    if [ ! -z "$run_pid" ] && kill -0 "$run_pid" 2>/dev/null; then
        echo "  mission_fsm multidrone_sim.launch already running (PID: $run_pid)"
        set_ego_planning_state false >/dev/null 2>&1
        return 0
    fi

    cd "$UAV_WS" || return 1
    source_uav_ws || return 1

    rm -f "$EGO_RUN_PID_FILE"
    roslaunch mission_fsm multidrone_sim.launch &
    RUN_IN_SIM_PID=$!
    echo $RUN_IN_SIM_PID > "$EGO_RUN_PID_FILE"
    echo "  mission_fsm multidrone_sim.launch started (PID: $RUN_IN_SIM_PID)"
    set_ego_planning_state false >/dev/null 2>&1

    cd "$PROJECT_ROOT" || return 1
}

ensure_ego_mapping_running() {
    local run_pid=""
    if [ -f "$EGO_RUN_PID_FILE" ]; then
        run_pid=$(cat "$EGO_RUN_PID_FILE" 2>/dev/null)
    fi

    if [ ! -z "$run_pid" ] && kill -0 "$run_pid" 2>/dev/null; then
        return 0
    fi

    echo "Ego mapping is not running, restarting..."
    start_ego_mapping
}

start_track() {
    echo "Starting tracker from uav-planner (standby)..."

    local track_pid=""
    if [ -f "$TRACK_PID_FILE" ]; then
        track_pid=$(cat "$TRACK_PID_FILE" 2>/dev/null)
    fi
    if [ ! -z "$track_pid" ] && kill -0 "$track_pid" 2>/dev/null; then
        echo "  tracker real_external.launch already running (PID: $track_pid)"
        return 0
    fi

    local yolo_topic="${YOLO_TOPIC:-/target/odom}"
    local odom_topic="${ODOM_TOPIC:-/track_ekf/ekf_odom}"
    local local_map_topic="${LOCAL_MAP_TOPIC:-/drone_0_ego_planner_node/grid_map/occupancy_inflate}"
    local track_trigger_topic="${TRACK_TRIGGER_TOPIC:-/tracker_trigger}"
    local track_preempt_topic="${TRACK_PREEMPT_TOPIC:-/tracker_preempt}"
    local track_mode_trigger_topic="${TRACK_MODE_TRIGGER_TOPIC:-/uav_planner/trigger}"
    local track_position_cmd_topic="${TRACK_POSITION_CMD_TOPIC:-/tracker_planning/pos_cmd}"
    local auto_track_trigger="${AUTO_TRACK_TRIGGER:-0}"

    cd "$UAV_WS" || return 1
    source_uav_ws || return 1

    rm -f "$TRACK_PID_FILE"
    roslaunch planning real_external.launch \
        yolo_topic:="$yolo_topic" \
        odom_topic:="$odom_topic" \
        local_map_topic:="$local_map_topic" \
        trigger_topic:="$track_trigger_topic" \
        preempt_topic:="$track_preempt_topic" \
        mode_trigger_topic:="$track_mode_trigger_topic" \
        position_cmd_topic:="$track_position_cmd_topic" &
    TRACK_PID=$!
    echo "$TRACK_PID" > "$TRACK_PID_FILE"
    echo "  planning real_external.launch started (PID: $TRACK_PID)"

    if [ "$auto_track_trigger" = "1" ]; then
        rostopic pub -1 "$track_trigger_topic" geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" >/dev/null 2>&1 || true
    fi

    cd "$PROJECT_ROOT" || return 1
}

ensure_track_process_running() {
    local track_pid=""
    if [ -f "$TRACK_PID_FILE" ]; then
        track_pid=$(cat "$TRACK_PID_FILE" 2>/dev/null)
    fi

    if [ ! -z "$track_pid" ] && kill -0 "$track_pid" 2>/dev/null; then
        return 0
    fi

    echo "Track process is not running, restarting..."
    start_track
}

start_perch() {
    echo "Starting perch..."
    local perch_cmd_topic="${PERCH_POSITION_CMD_TOPIC:-/drone_0_planning/pos_cmd}"

    cd "$PROJECT_ROOT/Fast-Perching" || return 1
    source devel/setup.sh

    rm -f "$PERCHING_PID_FILE"

    echo "Launching perching.launch (cmd topic: $perch_cmd_topic)..."
    roslaunch planning perching.launch \
        position_cmd_topic:="$perch_cmd_topic" &
    PERCHING_PID=$!
    echo $PERCHING_PID > "$PERCHING_PID_FILE"
    echo "  perching.launch started (PID: $PERCHING_PID)"

    cd "$PROJECT_ROOT" || return 1
}

ensure_perch_process_running() {
    local perch_pid=""
    if [ -f "$PERCHING_PID_FILE" ]; then
        perch_pid=$(cat "$PERCHING_PID_FILE" 2>/dev/null)
    fi

    if [ ! -z "$perch_pid" ] && kill -0 "$perch_pid" 2>/dev/null; then
        return 0
    fi

    echo "Perch process is not running, restarting..."
    start_perch
}

start_trigger_arbiter() {
    local arbiter_pid=""
    if [ -f "$ARBITER_PID_FILE" ]; then
        arbiter_pid=$(cat "$ARBITER_PID_FILE" 2>/dev/null)
    fi
    if [ ! -z "$arbiter_pid" ] && kill -0 "$arbiter_pid" 2>/dev/null; then
        return 0
    fi

    echo "Starting task trigger arbiter..."
    # Ensure no stale arbiter process from other workspaces keeps the same ROS node name.
    rosnode kill /task_trigger_arbiter >/dev/null 2>&1 || true
    pkill -f "task_trigger_arbiter.py" 2>/dev/null || true
    sleep 0.2

    cd "$UAV_WS" || return 1
    source_uav_ws || return 1
    cd "$PROJECT_ROOT" || return 1

    rm -f "$ARBITER_PID_FILE"
    python3 "$PROJECT_ROOT/utils/task_trigger_arbiter.py" &
    arbiter_pid=$!
    echo "$arbiter_pid" > "$ARBITER_PID_FILE"
    echo "  task_trigger_arbiter.py started (PID: $arbiter_pid)"
}

ensure_trigger_arbiter_running() {
    local arbiter_pid=""
    if [ -f "$ARBITER_PID_FILE" ]; then
        arbiter_pid=$(cat "$ARBITER_PID_FILE" 2>/dev/null)
    fi
    if [ ! -z "$arbiter_pid" ] && kill -0 "$arbiter_pid" 2>/dev/null; then
        return 0
    fi
    echo "Task trigger arbiter is not running, restarting..."
    start_trigger_arbiter
}

echo "1. Cleaning stale PID files..."
rm -f \
    "$START_PERCH_PID_FILE" \
    "$TRACK_PID_FILE" \
    "$EGO_RUN_PID_FILE" \
    "$PERCHING_PID_FILE" \
    "$ARBITER_PID_FILE"

echo "2. Starting ego mapping..."
start_ego_mapping
sleep 1

echo "3. Starting tracker runtime (standby)..."
start_track
sleep 1

echo "4. Starting perching runtime (standby)..."
start_perch
sleep 1

if [ "$USE_LEGACY_ARBITER" = "1" ]; then
    echo "5. Starting legacy trigger arbiter (compatibility mode)..."
    start_trigger_arbiter
    sleep 1
else
    echo "5. Skip legacy trigger arbiter; mode switching is in ego FSM (/uav_planner/trigger)."
fi

echo ""
echo "=== All runtimes are up (mode-trigger dispatcher in ego FSM) ==="
echo "Use tools/trigger_*.py to trigger ego/track/perch directly."
echo "Press Ctrl+C to stop all."
echo ""

while true; do
    ensure_ego_mapping_running
    ensure_track_process_running
    ensure_perch_process_running
    if [ "$USE_LEGACY_ARBITER" = "1" ]; then
        ensure_trigger_arbiter_running
    fi
    sleep 1
done
