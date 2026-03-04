#!/bin/bash
# stop_all.sh - stop all runtimes started by scripts/mix.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TMP_DIR="$PROJECT_ROOT/tmp"
MIX_PID_FILE="$TMP_DIR/mix.pid"

echo "=== stop_all.sh: stopping mix runtimes ==="

pid_exists() {
    local pid="$1"
    [ -n "$pid" ] && [[ "$pid" =~ ^[0-9]+$ ]] && ps -p "$pid" >/dev/null 2>&1
}

pid_owner() {
    local pid="$1"
    ps -o user= -p "$pid" 2>/dev/null | awk '{print $1}'
}

signal_with_fallback() {
    local signal_name="$1"
    local target="$2"
    if kill "-$signal_name" -- "$target" 2>/dev/null; then
        return 0
    fi
    if command -v sudo >/dev/null 2>&1; then
        if sudo -n kill "-$signal_name" -- "$target" 2>/dev/null; then
            return 0
        fi
    fi
    return 1
}

kill_pid_and_group() {
    local pid="$1"
    local label="$2"
    if [ -z "$pid" ] || ! [[ "$pid" =~ ^[0-9]+$ ]]; then
        return 0
    fi
    if ! pid_exists "$pid"; then
        return 0
    fi

    local pgid=""
    local owner=""
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    owner=$(pid_owner "$pid")
    echo "  stopping $label (PID: $pid${pgid:+, PGID: $pgid})"

    if ! signal_with_fallback "INT" "$pid"; then
        if [ -n "$owner" ] && [ "$owner" != "$(id -un)" ]; then
            echo "  warning: no permission to signal PID $pid (owner: $owner). Try: sudo ./scripts/stop_all.sh"
        fi
    fi
    if [ -n "$pgid" ]; then
        signal_with_fallback "INT" "-$pgid" || true
    fi
    sleep 0.5

    if pid_exists "$pid"; then
        signal_with_fallback "TERM" "$pid" || true
        if [ -n "$pgid" ]; then
            signal_with_fallback "TERM" "-$pgid" || true
        fi
        sleep 0.5
    fi

    if pid_exists "$pid"; then
        signal_with_fallback "KILL" "$pid" || true
        if [ -n "$pgid" ]; then
            signal_with_fallback "KILL" "-$pgid" || true
        fi
    fi
}

kill_pid_file_if_alive() {
    local pid_file="$1"
    local label="$2"
    if [ ! -f "$pid_file" ]; then
        return 0
    fi
    local pid
    pid=$(cat "$pid_file" 2>/dev/null)
    kill_pid_and_group "$pid" "$label"
    rm -f "$pid_file"
}

stop_mix_process() {
    echo "1) stop mix supervisor first"

    if [ -f "$MIX_PID_FILE" ]; then
        local mix_pid
        mix_pid=$(cat "$MIX_PID_FILE" 2>/dev/null)
        kill_pid_and_group "$mix_pid" "mix supervisor"
        rm -f "$MIX_PID_FILE"
    fi

    # Fallback for sessions started without mix.pid in this workspace.
    pkill -INT -f "$PROJECT_ROOT/scripts/mix.sh" 2>/dev/null || true
    pkill -INT -f "bash scripts/mix.sh" 2>/dev/null || true
    pkill -INT -f "sh scripts/mix.sh" 2>/dev/null || true
    sleep 0.5
    pkill -TERM -f "$PROJECT_ROOT/scripts/mix.sh" 2>/dev/null || true
    pkill -TERM -f "bash scripts/mix.sh" 2>/dev/null || true
    pkill -TERM -f "sh scripts/mix.sh" 2>/dev/null || true
    sleep 0.3
    pkill -KILL -f "$PROJECT_ROOT/scripts/mix.sh" 2>/dev/null || true
    pkill -KILL -f "bash scripts/mix.sh" 2>/dev/null || true
    pkill -KILL -f "sh scripts/mix.sh" 2>/dev/null || true

    # Cross-workspace fallback: stop any remaining */scripts/mix.sh process group.
    local mix_pids=()
    mapfile -t mix_pids < <(pgrep -f '(^| )bash +scripts/mix\.sh($| )|(^| )sh +scripts/mix\.sh($| )|/scripts/mix\.sh($| )' 2>/dev/null || true)
    for pid in "${mix_pids[@]}"; do
        kill_pid_and_group "$pid" "cross-workspace mix.sh"
    done
}

run_stop_helper() {
    local mode="$1"
    local setup_script="$2"

    if [ -f "$setup_script" ]; then
        (
            # shellcheck disable=SC1090
            source "$setup_script"
            python3 "$PROJECT_ROOT/utils/stop_task_helper.py" \
                --mode "$mode" \
                --tmp-dir "$TMP_DIR"
        ) >/dev/null 2>&1 || true
    else
        python3 "$PROJECT_ROOT/utils/stop_task_helper.py" \
            --mode "$mode" \
            --tmp-dir "$TMP_DIR" >/dev/null 2>&1 || true
    fi
}

stop_mix_process

# echo "2) run integrated stop helpers"
# run_stop_helper "ego" "$PROJECT_ROOT/ego-planner/devel/setup.sh"
# run_stop_helper "track" "$PROJECT_ROOT/Elastic-Tracker/devel/setup.sh"
# run_stop_helper "perch" "$PROJECT_ROOT/Fast-Perching/devel/setup.sh"

echo "3) kill residual processes by pid files"
kill_pid_file_if_alive "$MIX_PID_FILE" "mix.sh"
kill_pid_file_if_alive "$TMP_DIR/start_ego.pid" "start_ego.sh"
kill_pid_file_if_alive "$TMP_DIR/start_track.pid" "start_track.sh"
kill_pid_file_if_alive "$TMP_DIR/start_perch.pid" "start_perch.sh"
kill_pid_file_if_alive "$TMP_DIR/track_real.pid" "real_external.launch"
kill_pid_file_if_alive "$TMP_DIR/perching.pid" "perching.launch"
kill_pid_file_if_alive "$TMP_DIR/run_in_sim.pid" "multidrone_sim.launch"
kill_pid_file_if_alive "$TMP_DIR/task_trigger_arbiter.pid" "task_trigger_arbiter.py"
kill_pid_file_if_alive "$TMP_DIR/stop_script.pid" "stop helper"

echo "4) kill residual processes by pattern"
pkill -f "mission_fsm multidrone_sim.launch" 2>/dev/null || true
pkill -f "mission_fsm run_in_sim.xml" 2>/dev/null || true
pkill -f "planning real_external.launch" 2>/dev/null || true
pkill -f "perching.launch" 2>/dev/null || true
pkill -f "task_trigger_arbiter.py" 2>/dev/null || true
pkill -f "$PROJECT_ROOT/scripts/mix.sh" 2>/dev/null || true
pkill -f "bash scripts/mix.sh" 2>/dev/null || true
pkill -f "sh scripts/mix.sh" 2>/dev/null || true

echo "5) clear mix status files"
rm -f \
    "$MIX_PID_FILE" \
    "$TMP_DIR/first_execution.flag"

echo "=== stop_all.sh: done ==="
