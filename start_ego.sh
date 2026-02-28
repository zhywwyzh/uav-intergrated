#!/bin/bash
# Start ego node, get initialization parameters from temporary file

echo "=== Starting Ego Node ==="
echo ""

# PID file path
RUN_IN_SIM_PID_FILE="/tmp/run_in_sim.pid"
MAP_BRIDGE_PID_FILE="/tmp/map_generator.pid"
EGO_PLANNING_SERVICE="/drone_0_ego_planner_node/planning/enable"
# Shared temporary file path
POSITION_TMP_FILE="/tmp/drone_position.tmp"
MAP_BRIDGE_STARTED=false

# Cleanup function
cleanup() {
    echo ""
    echo "=== Received interrupt signal, starting cleanup... ==="
    
    if [ -f "$RUN_IN_SIM_PID_FILE" ]; then
        RUN_IN_SIM_PID=$(cat "$RUN_IN_SIM_PID_FILE" 2>/dev/null)
        if [ ! -z "$RUN_IN_SIM_PID" ] && kill -0 "$RUN_IN_SIM_PID" 2>/dev/null; then
            echo "  Terminating mission_fsm run_in_sim.xml (PID: $RUN_IN_SIM_PID)"
            kill -TERM "$RUN_IN_SIM_PID" 2>/dev/null
            kill -KILL "$RUN_IN_SIM_PID" 2>/dev/null
        fi
        rm -f "$RUN_IN_SIM_PID_FILE"
    fi

    if [ "$MAP_BRIDGE_STARTED" = true ] && [ -f "$MAP_BRIDGE_PID_FILE" ]; then
        MAP_BRIDGE_PID=$(cat "$MAP_BRIDGE_PID_FILE" 2>/dev/null)
        if [ ! -z "$MAP_BRIDGE_PID" ] && kill -0 "$MAP_BRIDGE_PID" 2>/dev/null; then
            echo "  Terminating livox_map_bridge.launch (PID: $MAP_BRIDGE_PID)"
            kill -TERM "$MAP_BRIDGE_PID" 2>/dev/null
            sleep 0.5
            kill -KILL "$MAP_BRIDGE_PID" 2>/dev/null
        fi
        rm -f "$MAP_BRIDGE_PID_FILE"
    fi
    
    pkill -f "mission_fsm run_in_sim.xml" 2>/dev/null
    
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
echo "2. Enabling planning via service..."

# Enter ego-planner directory and launch
cd ego-planner
source devel/setup.sh

# Start Livox map bridge if not started yet
if [ -f "$MAP_BRIDGE_PID_FILE" ]; then
    MAP_BRIDGE_PID=$(cat "$MAP_BRIDGE_PID_FILE" 2>/dev/null)
fi

if [ -z "$MAP_BRIDGE_PID" ] || ! kill -0 "$MAP_BRIDGE_PID" 2>/dev/null; then
    echo "  Starting livox_map_bridge.launch..."
    roslaunch mission_fsm livox_map_bridge.launch &
    MAP_BRIDGE_PID=$!
    echo "$MAP_BRIDGE_PID" > "$MAP_BRIDGE_PID_FILE"
    MAP_BRIDGE_STARTED=true
    sleep 1
fi

for retry in $(seq 1 20); do
    if rosservice call "$EGO_PLANNING_SERVICE" "data: true" >/dev/null 2>&1; then
        echo "  Planning enabled on existing ego mapping node"
        cd ..
        echo ""
        echo "=== ego-plan started ==="
        exit 0
    fi
    sleep 0.2
done

echo "  Service $EGO_PLANNING_SERVICE not available, fallback to launching run_in_sim.xml"

# Cleanup previous PID file
rm -f "$RUN_IN_SIM_PID_FILE"

# Launch mission_fsm run_in_sim.xml (EGO-Planner-v3)
roslaunch mission_fsm run_in_sim.xml \
    drone_id:=0 \
    map_size_x:=40.0 \
    map_size_y:=40.0 \
    map_size_z:=3.0 \
    init_x:="$INIT_X" \
    init_y:="$INIT_Y" \
    init_z:="$INIT_Z" \
    odom_topic:=/visual_slam/odom \
    cloud_topic:=/livox/lidar_register \
    flight_type:=1 \
    enable_planning:=true &
RUN_IN_SIM_PID=$!

echo $RUN_IN_SIM_PID > "$RUN_IN_SIM_PID_FILE"
echo "  mission_fsm run_in_sim.xml started (PID: $RUN_IN_SIM_PID)"

cd ..

echo ""
echo "=== ego-plan started ==="

# Wait for user to press Ctrl+C
wait
