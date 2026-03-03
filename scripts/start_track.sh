#!/bin/bash
# Start track node with direct external topics.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TMP_DIR="$PROJECT_ROOT/tmp"
mkdir -p "$TMP_DIR"

echo "=== Starting Track Node (Direct External Topics) ==="
echo ""

TRACK_PID_FILE="$TMP_DIR/track_real.pid"

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

YOLO_TOPIC="${YOLO_TOPIC:-/target/odom}"
ODOM_TOPIC="${ODOM_TOPIC:-/track_ekf/ekf_odom}"
LOCAL_MAP_TOPIC="${LOCAL_MAP_TOPIC:-/drone_0_ego_planner_node/grid_map/occupancy_inflate}"
TRACK_TRIGGER_TOPIC="${TRACK_TRIGGER_TOPIC:-/tracker_trigger}"
TRACK_PREEMPT_TOPIC="${TRACK_PREEMPT_TOPIC:-/tracker_preempt}"
TRACK_MODE_TRIGGER_TOPIC="${TRACK_MODE_TRIGGER_TOPIC:-/uav_planner/trigger}"
TRACK_POSITION_CMD_TOPIC="${TRACK_POSITION_CMD_TOPIC:-/drone_0_planning/pos_cmd}"
AUTO_TRACK_TRIGGER="${AUTO_TRACK_TRIGGER:-0}"

echo "1. Tracker topic configuration:"
echo "  Source YOLO topic : $YOLO_TOPIC"
echo "  Source Odom topic : $ODOM_TOPIC"
echo "  Local Map topic   : $LOCAL_MAP_TOPIC"
echo "  Track Trigger topic: $TRACK_TRIGGER_TOPIC"
echo "  Track Preempt topic: $TRACK_PREEMPT_TOPIC"
echo "  Mode Trigger topic: $TRACK_MODE_TRIGGER_TOPIC"
echo "  Track Cmd topic   : $TRACK_POSITION_CMD_TOPIC"

echo ""
echo "2. Launching Elastic-Tracker real_external.launch..."
cd "$PROJECT_ROOT/Elastic-Tracker" || exit 1
source devel/setup.sh

rm -f "$TRACK_PID_FILE"
roslaunch planning real_external.launch \
    yolo_topic:="$YOLO_TOPIC" \
    odom_topic:="$ODOM_TOPIC" \
    local_map_topic:="$LOCAL_MAP_TOPIC" \
    trigger_topic:="$TRACK_TRIGGER_TOPIC" \
    preempt_topic:="$TRACK_PREEMPT_TOPIC" \
    mode_trigger_topic:="$TRACK_MODE_TRIGGER_TOPIC" \
    position_cmd_topic:="$TRACK_POSITION_CMD_TOPIC" &
TRACK_PID=$!
echo "$TRACK_PID" > "$TRACK_PID_FILE"
echo "  real_external.launch started (PID: $TRACK_PID)"

cd "$PROJECT_ROOT"

if [ "$AUTO_TRACK_TRIGGER" = "1" ]; then
echo ""
echo "3. Publishing track trigger message..."
rostopic pub -1 "$TRACK_TRIGGER_TOPIC" geometry_msgs/PoseStamped "header:
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
echo "  Track trigger message published"
fi

echo ""
echo "=== Track started ==="

wait
