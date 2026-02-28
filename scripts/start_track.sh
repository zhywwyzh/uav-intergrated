#!/bin/bash
# Start track node with bridged track_object/track_odom topics.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TMP_DIR="$PROJECT_ROOT/tmp"
mkdir -p "$TMP_DIR"

echo "=== Starting Track Node (Bridge + External Topics) ==="
echo ""

TRACK_PID_FILE="$TMP_DIR/track_real.pid"
TRACK_BRIDGE_PID_FILE="$TMP_DIR/track_topic_bridge.pid"
TRACK_BRIDGE_SCRIPT="$PROJECT_ROOT/utils/track_topic_bridge.py"

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

    if [ -f "$TRACK_BRIDGE_PID_FILE" ]; then
        BRIDGE_PID=$(cat "$TRACK_BRIDGE_PID_FILE" 2>/dev/null)
        if [ ! -z "$BRIDGE_PID" ] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
            echo "  Terminating track topic bridge (PID: $BRIDGE_PID)"
            kill -TERM "$BRIDGE_PID" 2>/dev/null
            sleep 0.2
            kill -KILL "$BRIDGE_PID" 2>/dev/null
        fi
        rm -f "$TRACK_BRIDGE_PID_FILE"
    fi

    pkill -f "planning real_external.launch" 2>/dev/null
    pkill -f "track_topic_bridge.py" 2>/dev/null

    echo "=== Cleanup completed, script exiting ==="
    exit 1
}

trap cleanup SIGINT SIGTERM SIGQUIT

YOLO_TOPIC="${YOLO_TOPIC:-/yolov5trt/bboxes_pub}"
ODOM_TOPIC="${ODOM_TOPIC:-/ekf/ekf_odom}"
LOCAL_MAP_TOPIC="${LOCAL_MAP_TOPIC:-/drone_0_ego_planner_node/grid_map/occupancy_inflate}"
TRACK_OBJECT_TOPIC="${TRACK_OBJECT_TOPIC:-/track_object_topic}"
TRACK_ODOM_TOPIC="${TRACK_ODOM_TOPIC:-/track_odom_topic}"

echo "1. Tracker topic configuration:"
echo "  Source YOLO topic : $YOLO_TOPIC"
echo "  Source Odom topic : $ODOM_TOPIC"
echo "  Local Map topic   : $LOCAL_MAP_TOPIC"
echo "  Track Object topic: $TRACK_OBJECT_TOPIC"
echo "  Track Odom topic  : $TRACK_ODOM_TOPIC"

if [ ! -f "$TRACK_BRIDGE_SCRIPT" ]; then
    echo "Error: track bridge script not found: $TRACK_BRIDGE_SCRIPT"
    exit 1
fi

echo ""
echo "2. Launching track topic bridge..."
cd "$PROJECT_ROOT/Elastic-Tracker" || exit 1
source devel/setup.sh

rm -f "$TRACK_BRIDGE_PID_FILE"
python3 "$TRACK_BRIDGE_SCRIPT" \
    --source-yolo-topic "$YOLO_TOPIC" \
    --source-odom-topic "$ODOM_TOPIC" \
    --track-object-topic "$TRACK_OBJECT_TOPIC" \
    --track-odom-topic "$TRACK_ODOM_TOPIC" &
BRIDGE_PID=$!
echo "$BRIDGE_PID" > "$TRACK_BRIDGE_PID_FILE"
echo "  track_topic_bridge.py started (PID: $BRIDGE_PID)"

sleep 0.5
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    echo "Error: track topic bridge exited unexpectedly."
    rm -f "$TRACK_BRIDGE_PID_FILE"
    exit 1
fi

echo ""
echo "3. Launching Elastic-Tracker real_external.launch..."
rm -f "$TRACK_PID_FILE"
roslaunch planning real_external.launch \
    yolo_topic:="$TRACK_OBJECT_TOPIC" \
    odom_topic:="$TRACK_ODOM_TOPIC" \
    local_map_topic:="$LOCAL_MAP_TOPIC" &
TRACK_PID=$!
echo "$TRACK_PID" > "$TRACK_PID_FILE"
echo "  real_external.launch started (PID: $TRACK_PID)"

cd "$PROJECT_ROOT"

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
echo "=== Track started ==="

wait
