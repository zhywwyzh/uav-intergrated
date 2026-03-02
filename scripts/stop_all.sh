#!/bin/bash
# stop_all.sh - stop all runtimes started by scripts/mix.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TMP_DIR="$PROJECT_ROOT/tmp"
MIX_PID_FILE="$TMP_DIR/mix.pid"

echo "=== stop_all.sh: stopping mix runtimes ==="

kill_pid_file_if_alive() {
    local pid_file="$1"
    local label="$2"
    if [ ! -f "$pid_file" ]; then
        return 0
    fi
    local pid
    pid=$(cat "$pid_file" 2>/dev/null)
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        echo "  stopping $label (PID: $pid)"
        kill -TERM "$pid" 2>/dev/null || true
        sleep 0.2
        kill -KILL "$pid" 2>/dev/null || true
    fi
    rm -f "$pid_file"
}

stop_mix_process() {
    echo "1) stop mix supervisor first"

    if [ -f "$MIX_PID_FILE" ]; then
        local mix_pid
        mix_pid=$(cat "$MIX_PID_FILE" 2>/dev/null)
        if [ -n "$mix_pid" ] && kill -0 "$mix_pid" 2>/dev/null; then
            echo "  request mix shutdown (SIGINT), PID: $mix_pid"
            kill -INT "$mix_pid" 2>/dev/null || true
            sleep 0.8
            if kill -0 "$mix_pid" 2>/dev/null; then
                echo "  mix still alive, sending SIGTERM"
                kill -TERM "$mix_pid" 2>/dev/null || true
                sleep 0.5
            fi
            if kill -0 "$mix_pid" 2>/dev/null; then
                echo "  mix still alive, sending SIGKILL"
                kill -KILL "$mix_pid" 2>/dev/null || true
            fi
        fi
        rm -f "$MIX_PID_FILE"
    fi

    # Fallback for sessions started without mix.pid.
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

echo "2) run integrated stop helpers"
run_stop_helper "ego" "$PROJECT_ROOT/ego-planner/devel/setup.sh"
run_stop_helper "track" "$PROJECT_ROOT/Elastic-Tracker/devel/setup.sh"
run_stop_helper "perch" "$PROJECT_ROOT/Fast-Perching/devel/setup.sh"

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
