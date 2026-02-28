#!/bin/bash
# Start ego local mapping module only (planning disabled)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TMP_DIR="$PROJECT_ROOT/tmp"
mkdir -p "$TMP_DIR"

echo "=== Starting Ego Mapping Only ==="
echo ""

RUN_IN_SIM_PID_FILE="$TMP_DIR/run_in_sim.pid"
MAP_BRIDGE_PID_FILE="$TMP_DIR/map_generator.pid"
EGO_PLANNING_SERVICE="/drone_0_ego_planner_node/planning/enable"

cd "$PROJECT_ROOT/ego-planner" || exit 1
source devel/setup.sh

if [ -f "$MAP_BRIDGE_PID_FILE" ]; then
    MAP_BRIDGE_PID=$(cat "$MAP_BRIDGE_PID_FILE" 2>/dev/null)
fi

if [ -z "$MAP_BRIDGE_PID" ] || ! kill -0 "$MAP_BRIDGE_PID" 2>/dev/null; then
    echo "1. Starting livox_map_bridge.launch..."
    roslaunch mission_fsm livox_map_bridge.launch &
    MAP_BRIDGE_PID=$!
    echo "$MAP_BRIDGE_PID" > "$MAP_BRIDGE_PID_FILE"
    sleep 1
else
    echo "1. livox_map_bridge.launch already running (PID: $MAP_BRIDGE_PID)"
fi

if [ -f "$RUN_IN_SIM_PID_FILE" ]; then
    RUN_PID=$(cat "$RUN_IN_SIM_PID_FILE" 2>/dev/null)
fi

if [ ! -z "$RUN_PID" ] && kill -0 "$RUN_PID" 2>/dev/null; then
    echo "2. run_in_sim already running (PID: $RUN_PID), disabling planning..."
    rosservice call "$EGO_PLANNING_SERVICE" "data: false" >/dev/null 2>&1
    cd "$PROJECT_ROOT"
    echo ""
    echo "=== Ego Mapping Ready ==="
    exit 0
fi

echo "2. Launching run_in_sim.xml with enable_planning:=false..."
rm -f "$RUN_IN_SIM_PID_FILE"
roslaunch mission_fsm run_in_sim.xml \
    drone_id:=0 \
    map_size_x:=40.0 \
    map_size_y:=40.0 \
    map_size_z:=3.0 \
    init_x:=0.0 \
    init_y:=0.0 \
    init_z:=2.0 \
    odom_topic:=/visual_slam/odom \
    cloud_topic:=/livox/lidar_register \
    flight_type:=1 \
    enable_planning:=false &
RUN_PID=$!
echo "$RUN_PID" > "$RUN_IN_SIM_PID_FILE"
echo "  run_in_sim mapping-only started (PID: $RUN_PID)"

cd "$PROJECT_ROOT"
echo ""
echo "=== Ego Mapping Ready ==="
