#!/bin/bash
# Start ego node, get initialization parameters from temporary file - perching version

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TMP_DIR="$PROJECT_ROOT/tmp"
mkdir -p "$TMP_DIR"

echo "=== Starting Perch Node ==="
echo ""

# PID file path
PERCHING_PID_FILE="$TMP_DIR/perching.pid"
# Shared temporary file path
POSITION_TMP_FILE="$TMP_DIR/drone_position.tmp"

# Cleanup function
cleanup() {
    echo ""
    echo "=== Received interrupt signal, starting cleanup... ==="
    
    if [ -f "$PERCHING_PID_FILE" ]; then
        PERCHING_PID=$(cat "$PERCHING_PID_FILE" 2>/dev/null)
        if [ ! -z "$PERCHING_PID" ] && kill -0 "$PERCHING_PID" 2>/dev/null; then
            echo "  Terminating perching.launch (PID: $PERCHING_PID)"
            kill -TERM "$PERCHING_PID" 2>/dev/null
            kill -KILL "$PERCHING_PID" 2>/dev/null
        fi
        rm -f "$PERCHING_PID_FILE"
    fi
    
    pkill -f "perching.launch" 2>/dev/null
    
    echo "=== Cleanup completed, script exiting ==="
    exit 1
}

# Register signal handlers
trap cleanup SIGINT SIGTERM SIGQUIT

# 1. Get initialization parameters from temporary file
echo "1. Getting initialization parameters from temporary file..."

# Set default values
INIT_X="0.0"
INIT_Y="0.0"
INIT_Z="2.0"
INIT_YAW="0.0"

# Check if temporary file exists
if [ -f "$POSITION_TMP_FILE" ]; then
    echo "  Found shared temporary file: $POSITION_TMP_FILE"
    
    # Read parameters from file
    if source "$POSITION_TMP_FILE" 2>/dev/null; then
        # Check if parameters are valid
        if [ ! -z "$INIT_X" ] && [ ! -z "$INIT_Y" ] && [ ! -z "$INIT_Z" ]; then
            echo "  Parameters read successfully: x=$INIT_X, y=$INIT_Y, z=$INIT_Z, yaw=$INIT_YAW rad"
        else
            echo "  ⚠ File parameter format error, using default values"
        fi
    else
        # If source fails, try manual parsing
        echo "  Attempting to parse file manually..."
        INIT_X=$(grep "^INIT_X=" "$POSITION_TMP_FILE" | cut -d'=' -f2 2>/dev/null | head -1)
        INIT_Y=$(grep "^INIT_Y=" "$POSITION_TMP_FILE" | cut -d'=' -f2 2>/dev/null | head -1)
        INIT_Z=$(grep "^INIT_Z=" "$POSITION_TMP_FILE" | cut -d'=' -f2 2>/dev/null | head -1)
        INIT_YAW=$(grep "^INIT_YAW=" "$POSITION_TMP_FILE" | cut -d'=' -f2 2>/dev/null | head -1)
        
        # Use defaults if parameters are empty
        if [ -z "$INIT_X" ] || [ -z "$INIT_Y" ] || [ -z "$INIT_Z" ]; then
            echo "  ⚠ Parameters missing, using default values"
            INIT_X="0.0"
            INIT_Y="0.0"
            INIT_Z="2.0"
            INIT_YAW="0.0"
        else
            echo "  Parameters read successfully: x=$INIT_X, y=$INIT_Y, z=$INIT_Z, yaw=$INIT_YAW rad"
        fi
    fi
else
    echo "  ⚠ Temporary file $POSITION_TMP_FILE not found, using default values"
fi

echo ""
echo "=== Starting Perch Node ==="
echo "Startup parameters:"
echo "  Position: x=$INIT_X, y=$INIT_Y, z=$INIT_Z"
echo "  Orientation: yaw=$INIT_YAW rad"

echo ""
# Start new node
cd "$PROJECT_ROOT/Fast-Perching"
source devel/setup.sh

# Cleanup previous PID file
rm -f "$PERCHING_PID_FILE"

echo ""
echo "2. Launching perching.launch..."
roslaunch planning perching.launch \
    init_x:="$INIT_X" \
    init_y:="$INIT_Y" \
    init_z:="$INIT_Z" \
    init_yaw:="$INIT_YAW" &
PERCHING_PID=$!
echo $PERCHING_PID > "$PERCHING_PID_FILE"
echo "  perching.launch started (PID: $PERCHING_PID)"

cd "$PROJECT_ROOT"

echo ""
echo "=== Perching node started ==="

# Wait for user to press Ctrl+C
wait
