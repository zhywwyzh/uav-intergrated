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
TMP_DIR="$PROJECT_ROOT/tmp"
mkdir -p "$TMP_DIR"

echo "=== Main Control Script Starting (Direct-trigger mode) ==="
echo ""

# PID file paths
EGO_RUN_PID_FILE="$TMP_DIR/run_in_sim.pid"
TRACK_PID_FILE="$TMP_DIR/track_real.pid"
PERCHING_PID_FILE="$TMP_DIR/perching.pid"

# Startup script PID files
START_TRACK_PID_FILE="$TMP_DIR/start_track.pid"
START_PERCH_PID_FILE="$TMP_DIR/start_perch.pid"

EGO_PLANNING_SERVICE="/drone_0_ego_planner_node/planning/enable"

cleanup() {
    echo ""
    echo "=== Received interrupt signal, starting cleanup... ==="

    # stop startup wrappers
    if [ -f "$START_TRACK_PID_FILE" ]; then
        START_TRACK_PID=$(cat "$START_TRACK_PID_FILE" 2>/dev/null)
        if [ ! -z "$START_TRACK_PID" ] && kill -0 "$START_TRACK_PID" 2>/dev/null; then
            echo "  Terminating start_track.sh process (PID: $START_TRACK_PID)"
            kill -TERM "$START_TRACK_PID" 2>/dev/null
            sleep 0.2
            kill -KILL "$START_TRACK_PID" 2>/dev/null
        fi
        rm -f "$START_TRACK_PID_FILE"
    fi

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

    # call stop wrappers for robust ROS cleanup
    if [ -f "$SCRIPT_DIR/stop_track.sh" ]; then
        "$SCRIPT_DIR/stop_track.sh" >/dev/null 2>&1 || true
    fi
    if [ -f "$SCRIPT_DIR/stop_perch.sh" ]; then
        "$SCRIPT_DIR/stop_perch.sh" >/dev/null 2>&1 || true
    fi
    if [ -f "$SCRIPT_DIR/stop_ego.sh" ]; then
        "$SCRIPT_DIR/stop_ego.sh" >/dev/null 2>&1 || true
    fi

    # fallback kill by pattern
    pkill -f "mission_fsm multidrone_sim.launch" 2>/dev/null
    pkill -f "mission_fsm run_in_sim.xml" 2>/dev/null
    pkill -f "planning real_external.launch" 2>/dev/null
    pkill -f "perching.launch" 2>/dev/null

    rm -f \
        "$EGO_RUN_PID_FILE" \
        "$TRACK_PID_FILE" \
        "$PERCHING_PID_FILE"

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

start_ego_mapping() {
    echo "Starting ego-planner mapping (planning disabled)..."

    local run_pid=""
    if [ -f "$EGO_RUN_PID_FILE" ]; then
        run_pid=$(cat "$EGO_RUN_PID_FILE" 2>/dev/null)
    fi
    if [ ! -z "$run_pid" ] && kill -0 "$run_pid" 2>/dev/null; then
        echo "  mission_fsm multidrone_sim.launch already running (PID: $run_pid)"
        set_ego_planning_state false >/dev/null 2>&1
        return 0
    fi

    cd "$PROJECT_ROOT/ego-planner" || return 1
    source devel/setup.sh

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
    rm -f "$START_TRACK_PID_FILE"
    "$SCRIPT_DIR/start_track.sh" &
    START_TRACK_PID=$!
    echo $START_TRACK_PID > "$START_TRACK_PID_FILE"
    echo "  start_track.sh started (PID: $START_TRACK_PID)"
}

ensure_track_process_running() {
    local track_pid=""
    if [ -f "$TRACK_PID_FILE" ]; then
        track_pid=$(cat "$TRACK_PID_FILE" 2>/dev/null)
    fi

    if [ ! -z "$track_pid" ] && kill -0 "$track_pid" 2>/dev/null; then
        return 0
    fi

    # start wrapper may still be bringing up launch process
    local start_track_pid=""
    if [ -f "$START_TRACK_PID_FILE" ]; then
        start_track_pid=$(cat "$START_TRACK_PID_FILE" 2>/dev/null)
    fi
    if [ ! -z "$start_track_pid" ] && kill -0 "$start_track_pid" 2>/dev/null; then
        return 0
    fi

    echo "Track process is not running, restarting..."
    start_track
}

start_perch() {
    echo "Starting perch..."

    cd "$PROJECT_ROOT/Fast-Perching" || return 1
    source devel/setup.sh

    rm -f "$PERCHING_PID_FILE"

    echo "Launching perching.launch..."
    roslaunch planning perching.launch &
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

echo "1. Cleaning stale PID files..."
rm -f \
    "$START_TRACK_PID_FILE" \
    "$START_PERCH_PID_FILE" \
    "$TRACK_PID_FILE" \
    "$EGO_RUN_PID_FILE" \
    "$PERCHING_PID_FILE"

echo "2. Starting ego mapping..."
start_ego_mapping
sleep 1

echo "3. Starting tracker runtime (standby)..."
start_track
sleep 1

echo "4. Starting perching runtime (standby)..."
start_perch
sleep 1

echo ""
echo "=== All runtimes are up (direct trigger dispatcher) ==="
echo "Use tools/trigger_*.py to trigger ego/track/perch directly."
echo "Press Ctrl+C to stop all."
echo ""

while true; do
    ensure_ego_mapping_running
    ensure_track_process_running
    ensure_perch_process_running
    sleep 1
done
