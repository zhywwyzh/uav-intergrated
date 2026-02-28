#!/bin/bash
# Main control script: Start livox map bridge and call scripts based on /task_id messages

echo "=== Main Control Script Starting ==="
echo ""

# PID file paths
MAP_GENERATOR_PID_FILE="/tmp/map_generator.pid"
STOP_SCRIPT_PID_FILE="/tmp/stop_script.pid"
EGO_RUN_PID_FILE="/tmp/run_in_sim.pid"
EGO_PLANNING_SERVICE="/drone_0_ego_planner_node/planning/enable"

# Startup script PID files
START_EGO_PID_FILE="/tmp/start_ego.pid"
START_TRACK_PID_FILE="/tmp/start_track.pid"
START_TRACK_CAR_PID_FILE="/tmp/start_track_car.pid"
START_PERCH_PID_FILE="/tmp/start_perch.pid"

# Status marker files
LAST_TASK_ID_FILE="/tmp/last_task_id.txt"
FIRST_EXECUTION_FLAG="/tmp/first_execution.flag"

# Message deduplication variables
last_message_hash=""
last_processed_time=0
MIN_PROCESS_INTERVAL=0.5  # Minimum processing interval (seconds)

# Cleanup function
cleanup() {
    echo ""
    echo "=== Received interrupt signal, starting cleanup... ==="
    
    # Cleanup startup script processes
    if [ -f "$START_EGO_PID_FILE" ]; then
        START_EGO_PID=$(cat "$START_EGO_PID_FILE" 2>/dev/null)
        if [ ! -z "$START_EGO_PID" ] && kill -0 "$START_EGO_PID" 2>/dev/null; then
            echo "  Terminating start_ego.sh process (PID: $START_EGO_PID)"
            kill -TERM "$START_EGO_PID" 2>/dev/null
            sleep 0.5
            kill -KILL "$START_EGO_PID" 2>/dev/null 2>/dev/null
        fi
        rm -f "$START_EGO_PID_FILE"
    fi
    
    if [ -f "$START_TRACK_PID_FILE" ]; then
        START_TRACK_PID=$(cat "$START_TRACK_PID_FILE" 2>/dev/null)
        if [ ! -z "$START_TRACK_PID" ] && kill -0 "$START_TRACK_PID" 2>/dev/null; then
            echo "  Terminating start_track.sh process (PID: $START_TRACK_PID)"
            kill -TERM "$START_TRACK_PID" 2>/dev/null
            sleep 0.5
            kill -KILL "$START_TRACK_PID" 2>/dev/null 2>/dev/null
        fi
        rm -f "$START_TRACK_PID_FILE"
    fi
    
    if [ -f "$START_TRACK_CAR_PID_FILE" ]; then
        START_TRACK_CAR_PID=$(cat "$START_TRACK_CAR_PID_FILE" 2>/dev/null)
        if [ ! -z "$START_TRACK_CAR_PID" ] && kill -0 "$START_TRACK_CAR_PID" 2>/dev/null; then
            echo "  Terminating track_car.sh process (PID: $START_TRACK_CAR_PID)"
            kill -TERM "$START_TRACK_CAR_PID" 2>/dev/null
            sleep 0.5
            kill -KILL "$START_TRACK_CAR_PID" 2>/dev/null 2>/dev/null
        fi
        rm -f "$START_TRACK_CAR_PID_FILE"
    fi
    
    # Cleanup start_perch.sh process
    if [ -f "$START_PERCH_PID_FILE" ]; then
        START_PERCH_PID=$(cat "$START_PERCH_PID_FILE" 2>/dev/null)
        if [ ! -z "$START_PERCH_PID" ] && kill -0 "$START_PERCH_PID" 2>/dev/null; then
            echo "  Terminating start_perch.sh process (PID: $START_PERCH_PID)"
            kill -TERM "$START_PERCH_PID" 2>/dev/null
            sleep 0.5
            kill -KILL "$START_PERCH_PID" 2>/dev/null 2>/dev/null
        fi
        rm -f "$START_PERCH_PID_FILE"
    fi
    
    # Cleanup child script processes
    if [ -f "$STOP_SCRIPT_PID_FILE" ]; then
        STOP_SCRIPT_PID=$(cat "$STOP_SCRIPT_PID_FILE" 2>/dev/null)
        if [ ! -z "$STOP_SCRIPT_PID" ] && kill -0 "$STOP_SCRIPT_PID" 2>/dev/null 2>/dev/null; then
            echo "  Terminating child script process (PID: $STOP_SCRIPT_PID)"
            kill -TERM "$STOP_SCRIPT_PID" 2>/dev/null
            sleep 0.5
            kill -KILL "$STOP_SCRIPT_PID" 2>/dev/null 2>/dev/null
        fi
        rm -f "$STOP_SCRIPT_PID_FILE"
    fi
    
    # Cleanup livox map bridge process
    if [ -f "$MAP_GENERATOR_PID_FILE" ]; then
        MAP_PID=$(cat "$MAP_GENERATOR_PID_FILE" 2>/dev/null)
        if [ ! -z "$MAP_PID" ] && kill -0 "$MAP_PID" 2>/dev/null 2>/dev/null; then
            echo "  Terminating livox_map_bridge.launch (PID: $MAP_PID)"
            kill -TERM "$MAP_PID" 2>/dev/null
            sleep 0.5
            kill -KILL "$MAP_PID" 2>/dev/null 2>/dev/null
        fi
        rm -f "$MAP_GENERATOR_PID_FILE"
    fi
    
    # Cleanup perching related processes
    if [ -f "/tmp/perching.pid" ]; then
        PERCH_PID=$(cat "/tmp/perching.pid" 2>/dev/null)
        if [ ! -z "$PERCH_PID" ] && kill -0 "$PERCH_PID" 2>/dev/null; then
            echo "  Terminating perching.launch (PID: $PERCH_PID)"
            kill -TERM "$PERCH_PID" 2>/dev/null
            sleep 0.5
            kill -KILL "$PERCH_PID" 2>/dev/null 2>/dev/null
        fi
        rm -f "/tmp/perching.pid"
    fi
    
    # Cleanup possible related processes
    pkill -f "livox_map_bridge.launch" 2>/dev/null
    pkill -f "map_generator.launch" 2>/dev/null
    pkill -f "rostopic echo /task_id" 2>/dev/null
    pkill -f "perching.launch" 2>/dev/null
    pkill -f "planning real_external.launch" 2>/dev/null
    
    if [ -f "/tmp/track_real.pid" ]; then
        TRACK_PID=$(cat "/tmp/track_real.pid" 2>/dev/null)
        if [ ! -z "$TRACK_PID" ] && kill -0 "$TRACK_PID" 2>/dev/null 2>/dev/null; then
            echo "  Terminating real_external.launch (PID: $TRACK_PID)"
            kill -TERM "$TRACK_PID" 2>/dev/null
        fi
        rm -f "/tmp/track_real.pid"
    fi

    if [ -f "/tmp/track_car_real.pid" ]; then
        TRACK_CAR_PID=$(cat "/tmp/track_car_real.pid" 2>/dev/null)
        if [ ! -z "$TRACK_CAR_PID" ] && kill -0 "$TRACK_CAR_PID" 2>/dev/null 2>/dev/null; then
            echo "  Terminating real_external.launch (PID: $TRACK_CAR_PID)"
            kill -TERM "$TRACK_CAR_PID" 2>/dev/null
        fi
        rm -f "/tmp/track_car_real.pid"
    fi
    
    if [ -f "$EGO_RUN_PID_FILE" ]; then
        RUN_PID=$(cat "$EGO_RUN_PID_FILE" 2>/dev/null)
        if [ ! -z "$RUN_PID" ] && kill -0 "$RUN_PID" 2>/dev/null 2>/dev/null; then
            echo "  Terminating mission_fsm run_in_sim.xml (PID: $RUN_PID)"
            kill -TERM "$RUN_PID" 2>/dev/null
        fi
        rm -f "$EGO_RUN_PID_FILE"
    fi
    
    # Cleanup status files
    rm -f "$FIRST_EXECUTION_FLAG"
    
    # Clear last_task_id file (newly added)
    echo "Clearing last_task_id file..."
    rm -f "$LAST_TASK_ID_FILE"
    
    echo "=== Cleanup completed, script exiting ==="
    exit 0
}

# Register signal handlers
trap cleanup SIGINT SIGTERM SIGQUIT

# Startup helper functions
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
    echo "Starting ego mapping module (planning disabled)..."

    local run_pid=""
    if [ -f "$EGO_RUN_PID_FILE" ]; then
        run_pid=$(cat "$EGO_RUN_PID_FILE" 2>/dev/null)
    fi
    if [ ! -z "$run_pid" ] && kill -0 "$run_pid" 2>/dev/null; then
        echo "  mission_fsm run_in_sim.xml is already running (PID: $run_pid)"
        set_ego_planning_state false >/dev/null 2>&1
        return 0
    fi

    cd ego-planner
    source devel/setup.sh

    rm -f "$EGO_RUN_PID_FILE"
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
    RUN_IN_SIM_PID=$!
    echo $RUN_IN_SIM_PID > "$EGO_RUN_PID_FILE"
    echo "  mission_fsm run_in_sim.xml (mapping-only) started (PID: $RUN_IN_SIM_PID)"

    cd ..
}

start_ego_plan() {
    echo "Starting ego-plan..."
    start_ego_mapping
    set_ego_planning_state true
}

start_track() {
    echo "Starting track..."
    
    cd Elastic-Tracker
    source devel/setup.sh
    
    TRACK_PID_FILE="/tmp/track_real.pid"
    rm -f "$TRACK_PID_FILE"

    echo "Launching planning real_external.launch..."
    roslaunch planning real_external.launch \
        yolo_topic:=/yolov5trt/bboxes_pub \
        odom_topic:=/ekf/ekf_odom \
        global_map_topic:=/drone_0_ego_planner_node/grid_map/occupancy_inflate &
    TRACK_PID=$!
    echo "$TRACK_PID" > "$TRACK_PID_FILE"
    echo "  real_external.launch started (PID: $TRACK_PID)"
    
    # Publish trigger message
    echo "Publishing trigger message..."
    rostopic pub -1 /triger geometry_msgs/PoseStamped "{
  header: {
    seq: 0,
    stamp: {
      secs: 0,
      nsecs: 0
    },
    frame_id: ''
  },
  pose: {
    position: {
      x: 0.0,
      y: 0.0,
      z: 0.0
    },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      w: 0.0
    }
  }
}" >/dev/null 2>&1 
    
    echo "  Trigger message published"
    cd ..
}

start_track_car() {
    echo "Starting track-car..."
    
    cd Elastic-Tracker
    source devel/setup.sh
    
    TRACK_PID_FILE="/tmp/track_car_real.pid"
    rm -f "$TRACK_PID_FILE"

    echo "Launching planning real_external.launch..."
    roslaunch planning real_external.launch \
        yolo_topic:=/yolov5trt/bboxes_pub \
        odom_topic:=/ekf/ekf_odom \
        global_map_topic:=/drone_0_ego_planner_node/grid_map/occupancy_inflate &
    TRACK_PID=$!
    echo "$TRACK_PID" > "$TRACK_PID_FILE"
    echo "  real_external.launch started (PID: $TRACK_PID)"
    
    # Publish trigger message
    echo "Publishing trigger message..."
    rostopic pub -1 /triger geometry_msgs/PoseStamped "{
  header: {
    seq: 0,
    stamp: {
      secs: 0,
      nsecs: 0
    },
    frame_id: ''
  },
  pose: {
    position: {
      x: 0.0,
      y: 0.0,
      z: 0.0
    },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      w: 0.0
    }
  }
}" >/dev/null 2>&1 
    
    echo "  Trigger message published"
    cd ..
}

# New: Start perch function
start_perch() {
    echo "Starting perch..."
    
    cd Fast-Perching
    source devel/setup.sh
    
    # Cleanup previous PID file
    PERCHING_PID_FILE="/tmp/perching.pid"
    rm -f "$PERCHING_PID_FILE"
    
    # Launch with default parameters
    echo "Launching perching.launch..."
    roslaunch planning perching.launch &
    PERCHING_PID=$!
    echo $PERCHING_PID > "$PERCHING_PID_FILE"
    echo "  perching.launch started (PID: $PERCHING_PID)"
    
    cd ..
}

send_land_command() {
    echo "Sending land command..."
    
    echo "Executing: rostopic pub /land_triger geometry_msgs/PoseStamped ..."
    
    rostopic pub -1 /land_triger geometry_msgs/PoseStamped "header:
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
    w: 1.0" >/dev/null 2>&1
    
    echo "  Land command sent"
}

# 1. Start map bridge from Livox cloud
echo "1. Starting livox map bridge..."
cd ego-planner
source devel/setup.sh

# Cleanup previous PID files
rm -f "$MAP_GENERATOR_PID_FILE"
rm -f "$START_EGO_PID_FILE"
rm -f "$START_TRACK_PID_FILE"
rm -f "$START_TRACK_CAR_PID_FILE"
rm -f "$START_PERCH_PID_FILE"
rm -f "$EGO_RUN_PID_FILE"

roslaunch mission_fsm livox_map_bridge.launch &
MAP_GENERATOR_PID=$!
echo $MAP_GENERATOR_PID > "$MAP_GENERATOR_PID_FILE"
echo "  livox_map_bridge.launch started (PID: $MAP_GENERATOR_PID)"

cd ..
sleep 3

echo "1.5 Starting ego local mapping (planning will remain disabled)..."
start_ego_mapping
sleep 2

echo ""
echo "2. Starting to listen to /task_id messages..."
echo "   Waiting for task commands:"
echo "   Press Ctrl+C to exit script"
echo ""

# Initialize last task ID
last_task_id=""
if [ -f "$LAST_TASK_ID_FILE" ]; then
    last_task_id=$(cat "$LAST_TASK_ID_FILE" 2>/dev/null)
fi

# Set initial state to first execution
rm -f "$FIRST_EXECUTION_FLAG"
touch "$FIRST_EXECUTION_FLAG"
first_execution=true

# Loop to listen to /task_id messages
while true; do
    # Listen to /task_id topic, wait for new messages
    echo "Listening to /task_id messages..."
    
    # Use rostopic echo to get latest message (10 second timeout)
    task_msg=$(timeout 10 rostopic echo -n 1 /task_id 2>/dev/null)
    
    if [ $? -eq 0 ] && [ ! -z "$task_msg" ]; then
        # Calculate message hash
        current_hash=$(echo "$task_msg" | md5sum | cut -d' ' -f1)
        
        # Check processing interval
        current_time=$(date +%s.%N)
        time_diff=$(echo "$current_time - $last_processed_time" | bc)
        
        # Check if duplicate message (same hash) and processing interval too short
        if [ "$current_hash" != "$last_message_hash" ] || \
           [ $(echo "$time_diff >= $MIN_PROCESS_INTERVAL" | bc -l) -eq 1 ]; then
            last_message_hash="$current_hash"
            last_processed_time=$current_time
            
            # Extract task_id value
            current_task_id=$(echo "$task_msg" | grep "data:" | awk '{print $2}' | tr -d '\n\r')
            
            if [ ! -z "$current_task_id" ]; then
                echo ""
                echo "=== Received new task command: task_id=$current_task_id ==="
                echo "Last task ID: $last_task_id"
                
                # Check if task switching is needed
                if [ ! -z "$last_task_id" ] && [ "$last_task_id" != "$current_task_id" ]; then
                    echo "Detected task switch: $last_task_id -> $current_task_id"
                    
                    # Only execute stop script when not switching from 3 to 4
                    if [ "$last_task_id" = "3" ] && [ "$current_task_id" = "4" ]; then
                        echo "Switching from track-car to land command, skipping stop script"
                    else
                        # Execute stop script based on last task ID
                        if [ "$last_task_id" = "1" ]; then
                            # Cleanup old stop script PID file
                            rm -f "$STOP_SCRIPT_PID_FILE"
                            
                            echo "Executing stop_ego.sh..."
                            if [ -f "./stop_ego.sh" ]; then
                                ./stop_ego.sh &
                                STOP_SCRIPT_PID=$!
                                echo $STOP_SCRIPT_PID > "$STOP_SCRIPT_PID_FILE"
                                echo "  stop_ego.sh started (PID: $STOP_SCRIPT_PID)"
                                
                                # Wait for stop script to complete
                                wait $STOP_SCRIPT_PID 2>/dev/null
                                echo "stop_ego.sh execution completed"
                            else
                                echo "Warning: stop_ego.sh script not found"
                            fi
                            
                        elif [ "$last_task_id" = "2" ]; then
                            # Cleanup old stop script PID file
                            rm -f "$STOP_SCRIPT_PID_FILE"
                            
                            echo "Executing stop_track.sh..."
                            if [ -f "./stop_track.sh" ]; then
                                ./stop_track.sh &
                                STOP_SCRIPT_PID=$!
                                echo $STOP_SCRIPT_PID > "$STOP_SCRIPT_PID_FILE"
                                echo "  stop_track.sh started (PID: $STOP_SCRIPT_PID)"
                                
                                # Wait for stop script to complete
                                wait $STOP_SCRIPT_PID 2>/dev/null
                                echo "stop_track.sh execution completed"
                            else
                                echo "Warning: stop_track.sh script not found"
                            fi
                            
                        elif [ "$last_task_id" = "3" ]; then
                            # Cleanup old stop script PID file
                            rm -f "$STOP_SCRIPT_PID_FILE"
                            
                            echo "Executing stop_track.sh ..."
                            if [ -f "./stop_track.sh" ]; then
                                ./stop_track.sh &
                                STOP_SCRIPT_PID=$!
                                echo $STOP_SCRIPT_PID > "$STOP_SCRIPT_PID_FILE"
                                echo "  stop_track.sh started (PID: $STOP_SCRIPT_PID)"
                                
                                # Wait for stop script to complete
                                wait $STOP_SCRIPT_PID 2>/dev/null
                                echo "stop_track.sh execution completed"
                            else
                                echo "Warning: stop_track.sh script not found"
                            fi
                        elif [ "$last_task_id" = "5" ]; then
                            # Cleanup old stop script PID file
                            rm -f "$STOP_SCRIPT_PID_FILE"
                            
                            echo "Executing stop_perch.sh..."
                            if [ -f "./stop_perch.sh" ]; then
                                ./stop_perch.sh &
                                STOP_SCRIPT_PID=$!
                                echo $STOP_SCRIPT_PID > "$STOP_SCRIPT_PID_FILE"
                                echo "  stop_perch.sh started (PID: $STOP_SCRIPT_PID)"
                                
                                # Wait for stop script to complete
                                wait $STOP_SCRIPT_PID 2>/dev/null
                                echo "stop_perch.sh execution completed"
                            else
                                echo "Warning: stop_perch.sh script not found"
                            fi
                        fi
                        
                        # Cleanup stop script PID file
                        rm -f "$STOP_SCRIPT_PID_FILE"
                    fi
                fi
                
                # Start corresponding system or execute land command based on current task ID
                if [ "$current_task_id" = "1" ]; then
                    echo "Executing task: Start ego-plan"
                    
                    if [ "$first_execution" = true ]; then
                        echo "First execution, directly starting ego-plan..."
                        start_ego_plan
                        first_execution=false
                        rm -f "$FIRST_EXECUTION_FLAG"
                    else
                        echo "Not first execution, checking if startup needed..."
                        if [ "$last_task_id" != "1" ]; then
                            echo "Switching from other task, starting ego-plan..."
                            # Start start_ego.sh and record PID
                            rm -f "$START_EGO_PID_FILE"
                            ./start_ego.sh &
                            START_EGO_PID=$!
                            echo $START_EGO_PID > "$START_EGO_PID_FILE"
                            echo "  start_ego.sh started (PID: $START_EGO_PID)"
                        else
                            echo "Already in ego-plan task, skipping startup"
                        fi
                    fi
                    
                    # Save current task ID
                    echo "$current_task_id" > "$LAST_TASK_ID_FILE"
                    last_task_id="$current_task_id"
                    
                elif [ "$current_task_id" = "2" ]; then
                    echo "Executing task: Start track"
                    
                    if [ "$first_execution" = true ]; then
                        echo "First execution, directly starting track..."
                        start_track
                        first_execution=false
                        rm -f "$FIRST_EXECUTION_FLAG"
                    else
                        echo "Not first execution, checking if startup needed..."
                        if [ "$last_task_id" != "2" ]; then
                            echo "Switching from other task, starting track..."
                            # Start start_track.sh and record PID
                            rm -f "$START_TRACK_PID_FILE"
                            ./start_track.sh &
                            START_TRACK_PID=$!
                            echo $START_TRACK_PID > "$START_TRACK_PID_FILE"
                            echo "  start_track.sh started (PID: $START_TRACK_PID)"
                        else
                            echo "Already in track task, skipping startup"
                        fi
                    fi
                    
                    # Save current task ID
                    echo "$current_task_id" > "$LAST_TASK_ID_FILE"
                    last_task_id="$current_task_id"
                    
                elif [ "$current_task_id" = "3" ]; then
                    echo "Executing task: Start track-car"
                    
                    if [ "$first_execution" = true ]; then
                        echo "First execution, directly starting track-car..."
                        start_track_car
                        first_execution=false
                        rm -f "$FIRST_EXECUTION_FLAG"
                    else
                        echo "Not first execution, checking if startup needed..."
                        if [ "$last_task_id" != "3" ]; then
                            echo "Switching from other task, starting track-car..."
                            # Start start_track_car.sh and record PID
                            rm -f "$START_TRACK_CAR_PID_FILE"
                            ./start_track_car.sh &
                            START_TRACK_CAR_PID=$!
                            echo $START_TRACK_CAR_PID > "$START_TRACK_CAR_PID_FILE"
                            echo "  start_track_car.sh started (PID: $START_TRACK_CAR_PID)"
                        else
                            echo "Already in start_track-car task, skipping startup"
                        fi
                    fi
                    
                    # Save current task ID
                    echo "$current_task_id" > "$LAST_TASK_ID_FILE"
                    last_task_id="$current_task_id"
                    
                elif [ "$current_task_id" = "4" ]; then
                    echo "Executing task: Send land command"
                    
                    # Execute land command
                    send_land_command
                    
                    # Don't update last_task_id, maintain original task state
                    echo "Note: Land command sent, task status remains unchanged"
                    echo "Currently still in task: $last_task_id"
                    
                elif [ "$current_task_id" = "5" ]; then
                    echo "Executing task: Start perch"
                    
                    if [ "$first_execution" = true ]; then
                        echo "First execution, directly starting perch..."
                        start_perch
                        first_execution=false
                        rm -f "$FIRST_EXECUTION_FLAG"
                    else
                        echo "Not first execution, checking if startup needed..."
                        if [ "$last_task_id" != "5" ]; then
                            echo "Switching from other task, starting perch..."
                            # Start start_perch.sh and record PID
                            rm -f "$START_PERCH_PID_FILE"
                            ./start_perch.sh &
                            START_PERCH_PID=$!
                            echo $START_PERCH_PID > "$START_PERCH_PID_FILE"
                            echo "  start_perch.sh started (PID: $START_PERCH_PID)"
                        else
                            echo "Already in perch task, skipping startup"
                        fi
                    fi
                    
                    # Save current task ID
                    echo "$current_task_id" > "$LAST_TASK_ID_FILE"
                    last_task_id="$current_task_id"
                    
                else
                    echo "Unknown task_id: $current_task_id, ignoring this message"
                fi
                
                echo "=== Task execution completed, continuing to listen... ==="
                echo ""
            fi
        else
            echo "Ignoring duplicate or high-frequency message (hash: $current_hash)"
            # Short delay to avoid high CPU usage
            sleep 0.1
        fi
    else
        # Timeout or no message, continue listening
        sleep 0.5
    fi
done

# Script normally won't reach here
cleanup
