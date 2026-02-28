#!/bin/bash
# stop_all.sh - stop all runtimes started by scripts/mix.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TMP_DIR="$PROJECT_ROOT/tmp"

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

echo "1) request mix shutdown (SIGINT)"
pkill -INT -f "$PROJECT_ROOT/scripts/mix.sh" 2>/dev/null || true
sleep 1

echo "2) run module stop helpers"
if [ -x "$SCRIPT_DIR/stop_ego.sh" ]; then
    "$SCRIPT_DIR/stop_ego.sh" >/dev/null 2>&1 || true
fi
if [ -x "$SCRIPT_DIR/stop_track.sh" ]; then
    "$SCRIPT_DIR/stop_track.sh" >/dev/null 2>&1 || true
fi
if [ -x "$SCRIPT_DIR/stop_perch.sh" ]; then
    "$SCRIPT_DIR/stop_perch.sh" >/dev/null 2>&1 || true
fi

echo "3) kill residual processes by pid files"
kill_pid_file_if_alive "$TMP_DIR/start_ego.pid" "start_ego.sh"
kill_pid_file_if_alive "$TMP_DIR/start_track.pid" "start_track.sh"
kill_pid_file_if_alive "$TMP_DIR/start_perch.pid" "start_perch.sh"
kill_pid_file_if_alive "$TMP_DIR/track_real.pid" "real_external.launch"
kill_pid_file_if_alive "$TMP_DIR/perching.pid" "perching.launch"
kill_pid_file_if_alive "$TMP_DIR/run_in_sim.pid" "multidrone_sim.launch"
kill_pid_file_if_alive "$TMP_DIR/stop_script.pid" "stop helper"

echo "4) kill residual processes by pattern"
pkill -f "mission_fsm multidrone_sim.launch" 2>/dev/null || true
pkill -f "mission_fsm run_in_sim.xml" 2>/dev/null || true
pkill -f "planning real_external.launch" 2>/dev/null || true
pkill -f "perching.launch" 2>/dev/null || true
pkill -f "$PROJECT_ROOT/scripts/mix.sh" 2>/dev/null || true

echo "5) clear mix status files"
rm -f \
    "$TMP_DIR/first_execution.flag"

echo "=== stop_all.sh: done ==="
