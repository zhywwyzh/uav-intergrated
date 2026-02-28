#!/bin/bash
# Start track-car node with external odom/map/yolo topics (no legacy simulator)

echo "=== Starting Track-Car Node (External Topics) ==="
echo ""

TRACK_PID_FILE="/tmp/track_car_real.pid"
MAP_BRIDGE_PID_FILE="/tmp/map_generator.pid"

cleanup() {
    echo ""
    echo "=== Received interrupt signal, starting cleanup... ==="

    if [ -f "$TRACK_PID_FILE" ]; then
        TRACK_PID=$(cat "$TRACK_PID_FILE" 2>/dev/null)
        if [ ! -z "$TRACK_PID" ] && kill -0 "$TRACK_PID" 2>/dev/null; then
            echo "  Terminating planning real_external.launch (PID: $TRACK_PID)"
            kill -TERM "$TRACK_PID" 2>/dev/null
            sleep 0.5
            kill -KILL "$TRACK_PID" 2>/dev/null
        fi
        rm -f "$TRACK_PID_FILE"
    fi

    pkill -f "planning real_external.launch" 2>/dev/null

    echo "=== Cleanup completed, script exiting ==="
    exit 1
}

trap cleanup SIGINT SIGTERM SIGQUIT

ensure_map_bridge() {
    local map_pid=""
    if [ -f "$MAP_BRIDGE_PID_FILE" ]; then
        map_pid=$(cat "$MAP_BRIDGE_PID_FILE" 2>/dev/null)
    fi

    if [ -z "$map_pid" ] || ! kill -0 "$map_pid" 2>/dev/null; then
        echo "  Starting livox_map_bridge.launch for shared map topics..."
        cd ego-planner || return 1
        source devel/setup.sh
        roslaunch mission_fsm livox_map_bridge.launch &
        map_pid=$!
        echo "$map_pid" > "$MAP_BRIDGE_PID_FILE"
        cd ..
        sleep 1
    fi
}

YOLO_TOPIC="${YOLO_TOPIC:-/yolov5trt/bboxes_pub}"
ODOM_TOPIC="${ODOM_TOPIC:-/ekf/ekf_odom}"
GLOBAL_MAP_TOPIC="${GLOBAL_MAP_TOPIC:-/drone_0_ego_planner_node/grid_map/occupancy_inflate}"

echo "1. Track-car topic configuration:"
echo "  YOLO topic      : $YOLO_TOPIC"
echo "  Odom topic      : $ODOM_TOPIC"
echo "  Map topic       : $GLOBAL_MAP_TOPIC"

echo ""
echo "2. Ensuring Livox map bridge is running..."
ensure_map_bridge

echo ""
echo "3. Launching Elastic-Tracker real_external.launch..."
cd Elastic-Tracker || exit 1
source devel/setup.sh

rm -f "$TRACK_PID_FILE"
roslaunch planning real_external.launch \
    yolo_topic:="$YOLO_TOPIC" \
    odom_topic:="$ODOM_TOPIC" \
    global_map_topic:="$GLOBAL_MAP_TOPIC" &
TRACK_PID=$!
echo "$TRACK_PID" > "$TRACK_PID_FILE"
echo "  real_external.launch started (PID: $TRACK_PID)"

cd ..

echo ""
echo "4. Publishing trigger message..."
rostopic pub -1 /triger geometry_msgs/PoseStamped "header:
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
    w: 0.0"
echo "  Trigger message published"

echo ""
echo "=== Track-Car started ==="

wait
