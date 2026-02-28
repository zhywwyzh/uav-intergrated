#!/bin/bash
# Main control script: Start ego-planner and call scripts based on /task_id messages

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TMP_DIR="$PROJECT_ROOT/tmp"
mkdir -p "$TMP_DIR"

echo "=== Main Control Script Starting ==="
echo ""

# PID file paths
STOP_SCRIPT_PID_FILE="$TMP_DIR/stop_script.pid"
EGO_RUN_PID_FILE="$TMP_DIR/run_in_sim.pid"
EGO_PLANNING_SERVICE="/drone_0_ego_planner_node/planning/enable"
TRACK_PID_FILE="$TMP_DIR/track_real.pid"
TRACK_BRIDGE_PID_FILE="$TMP_DIR/track_topic_bridge.pid"
PERCHING_PID_FILE="$TMP_DIR/perching.pid"

# Startup script PID files
START_EGO_PID_FILE="$TMP_DIR/start_ego.pid"
START_TRACK_PID_FILE="$TMP_DIR/start_track.pid"
START_PERCH_PID_FILE="$TMP_DIR/start_perch.pid"

# Status marker files
LAST_TASK_ID_FILE="$TMP_DIR/last_task_id.txt"
FIRST_EXECUTION_FLAG="$TMP_DIR/first_execution.flag"
PENDING_TASK_ID_FILE="$TMP_DIR/pending_task_id.txt"

# Message deduplication variables
last_message_hash=""

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
    
    # Cleanup perching related processes
    if [ -f "$PERCHING_PID_FILE" ]; then
        PERCH_PID=$(cat "$PERCHING_PID_FILE" 2>/dev/null)
        if [ ! -z "$PERCH_PID" ] && kill -0 "$PERCH_PID" 2>/dev/null; then
            echo "  Terminating perching.launch (PID: $PERCH_PID)"
            kill -TERM "$PERCH_PID" 2>/dev/null
            sleep 0.5
            kill -KILL "$PERCH_PID" 2>/dev/null 2>/dev/null
        fi
        rm -f "$PERCHING_PID_FILE"
    fi
    
    # Cleanup possible related processes
    pkill -f "rostopic echo /task_id" 2>/dev/null
    pkill -f "perching.launch" 2>/dev/null
    pkill -f "planning real_external.launch" 2>/dev/null
    pkill -f "track_topic_bridge.py" 2>/dev/null
    pkill -f "mission_fsm multidrone_sim.launch" 2>/dev/null
    pkill -f "mission_fsm run_in_sim.xml" 2>/dev/null
    
    if [ -f "$TRACK_PID_FILE" ]; then
        TRACK_PID=$(cat "$TRACK_PID_FILE" 2>/dev/null)
        if [ ! -z "$TRACK_PID" ] && kill -0 "$TRACK_PID" 2>/dev/null 2>/dev/null; then
            echo "  Terminating real_external.launch (PID: $TRACK_PID)"
            kill -TERM "$TRACK_PID" 2>/dev/null
        fi
        rm -f "$TRACK_PID_FILE"
    fi

    if [ -f "$TRACK_BRIDGE_PID_FILE" ]; then
        TRACK_BRIDGE_PID=$(cat "$TRACK_BRIDGE_PID_FILE" 2>/dev/null)
        if [ ! -z "$TRACK_BRIDGE_PID" ] && kill -0 "$TRACK_BRIDGE_PID" 2>/dev/null 2>/dev/null; then
            echo "  Terminating track topic bridge (PID: $TRACK_BRIDGE_PID)"
            kill -TERM "$TRACK_BRIDGE_PID" 2>/dev/null
        fi
        rm -f "$TRACK_BRIDGE_PID_FILE"
    fi

    if [ -f "$EGO_RUN_PID_FILE" ]; then
        RUN_PID=$(cat "$EGO_RUN_PID_FILE" 2>/dev/null)
        if [ ! -z "$RUN_PID" ] && kill -0 "$RUN_PID" 2>/dev/null 2>/dev/null; then
            echo "  Terminating mission_fsm multidrone_sim.launch (PID: $RUN_PID)"
            kill -TERM "$RUN_PID" 2>/dev/null
        fi
        rm -f "$EGO_RUN_PID_FILE"
    fi
    
    # Cleanup status files
    rm -f "$FIRST_EXECUTION_FLAG"
    
    # Clear last_task_id file (newly added)
    echo "Clearing last_task_id file..."
    rm -f "$LAST_TASK_ID_FILE"
    rm -f "$PENDING_TASK_ID_FILE"
    
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
    echo "Starting ego-planner (multidrone_sim, planning disabled)..."

    local run_pid=""
    if [ -f "$EGO_RUN_PID_FILE" ]; then
        run_pid=$(cat "$EGO_RUN_PID_FILE" 2>/dev/null)
    fi
    if [ ! -z "$run_pid" ] && kill -0 "$run_pid" 2>/dev/null; then
        echo "  mission_fsm multidrone_sim.launch is already running (PID: $run_pid)"
        set_ego_planning_state false >/dev/null 2>&1
        return 0
    fi

    cd "$PROJECT_ROOT/ego-planner"
    source devel/setup.sh

    rm -f "$EGO_RUN_PID_FILE"
    roslaunch mission_fsm multidrone_sim.launch &
    RUN_IN_SIM_PID=$!
    echo $RUN_IN_SIM_PID > "$EGO_RUN_PID_FILE"
    echo "  mission_fsm multidrone_sim.launch started (PID: $RUN_IN_SIM_PID)"
    set_ego_planning_state false >/dev/null 2>&1

    cd "$PROJECT_ROOT"
}

start_ego_plan() {
    echo "Starting ego-plan..."
    start_ego_mapping
    set_ego_planning_state true
}

wait_and_forward_goalset_origin() {
    local source_topic="/goal_with_id_from_station_origin"
    local target_topic="/goal_with_id_from_station"
    local expected_type="quadrotor_msgs/GoalSet"

    echo "Waiting for GoalSet on $source_topic ..."

    while true; do
        # Interrupt waiting immediately if any new task command arrives.
        local new_task_msg
        new_task_msg=$(timeout 0.2 rostopic echo -n 1 /task_id 2>/dev/null)
        if [ $? -eq 0 ] && [ ! -z "$new_task_msg" ]; then
            local new_task_id
            new_task_id=$(echo "$new_task_msg" | grep "data:" | awk '{print $2}' | tr -d '\n\r')
            if [ ! -z "$new_task_id" ]; then
                echo "$new_task_id" > "$PENDING_TASK_ID_FILE"
                echo "  Detected new task_id=$new_task_id. Interrupting GoalSet wait."
                return 2
            fi
        fi

        local topic_type
        topic_type=$(rostopic type "$source_topic" 2>/dev/null | tr -d '\r')

        if [ -z "$topic_type" ]; then
            echo "  Topic $source_topic not found yet, waiting..."
            sleep 1
            continue
        fi

        if [ "$topic_type" != "$expected_type" ]; then
            echo "  Error: $source_topic type is '$topic_type', expected '$expected_type'."
            echo "  Topic format does not match the required GoalSet standard."
            sleep 1
            continue
        fi

        local goal_msg
        goal_msg=$(timeout 10 rostopic echo -n 1 "$source_topic" 2>/dev/null | sed '/^---$/d')
        if [ -z "$goal_msg" ]; then
            echo "  No message received on $source_topic yet, waiting..."
            continue
        fi

        # Basic message-field check to ensure message structure is GoalSet-like
        if ! echo "$goal_msg" | grep -q "to_drone_ids:" || \
           ! echo "$goal_msg" | grep -q "goal:" || \
           ! echo "$goal_msg" | grep -q "yaw:" || \
           ! echo "$goal_msg" | grep -q "look_forward:" || \
           ! echo "$goal_msg" | grep -q "goal_to_follower:"; then
            echo "  Error: message received on $source_topic does not match GoalSet standard fields."
            continue
        fi

        echo "  Valid GoalSet received. Forwarding to $target_topic ..."
        if rostopic pub -1 "$target_topic" "$expected_type" "$goal_msg" >/dev/null 2>&1; then
            echo "  GoalSet forwarded successfully."
            return 0
        fi

        echo "  Warning: failed to publish GoalSet to $target_topic, retrying..."
        sleep 1
    done
}

start_track() {
    rm -f "$START_TRACK_PID_FILE"
    "$SCRIPT_DIR/start_track.sh" &
    START_TRACK_PID=$!
    echo $START_TRACK_PID > "$START_TRACK_PID_FILE"
    echo "  start_track.sh started (PID: $START_TRACK_PID)"
}

ensure_track_process_running() {
    local track_pid=""
    if [ -f "$TRACK_PID_FILE" ]; then
        track_pid=$(cat "$TRACK_PID_FILE" 2>/dev/null)
    fi

    if [ ! -z "$track_pid" ] && kill -0 "$track_pid" 2>/dev/null; then
        return 0
    fi

    echo "Track process not running under task_id=2, restarting..."
    start_track
}

# New: Start perch function
start_perch() {
    echo "Starting perch..."
    
    cd "$PROJECT_ROOT/Fast-Perching"
    source devel/setup.sh

    # Cleanup previous PID file
    rm -f "$PERCHING_PID_FILE"
    
    # Launch with default parameters
    echo "Launching perching.launch..."
    roslaunch planning perching.launch &
    PERCHING_PID=$!
    echo $PERCHING_PID > "$PERCHING_PID_FILE"
    echo "  perching.launch started (PID: $PERCHING_PID)"
    
    cd "$PROJECT_ROOT"
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

# 1. Cleanup previous PID files
echo "1. Cleaning stale PID files..."
rm -f "$START_EGO_PID_FILE"
rm -f "$START_TRACK_PID_FILE"
rm -f "$START_PERCH_PID_FILE"
rm -f "$TRACK_PID_FILE"
rm -f "$TRACK_BRIDGE_PID_FILE"
rm -f "$EGO_RUN_PID_FILE"
rm -f "$LAST_TASK_ID_FILE"
rm -f "$PENDING_TASK_ID_FILE"

echo "2. Starting ego-planner local mapping (planning remains disabled until task_id=1)..."
start_ego_mapping
sleep 2

echo ""
echo "3. Starting to listen to /task_id messages..."
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
    task_msg_from_pending=false

    if [ -f "$PENDING_TASK_ID_FILE" ]; then
        pending_task_id=$(cat "$PENDING_TASK_ID_FILE" 2>/dev/null | tr -d '\n\r')
        if [ ! -z "$pending_task_id" ]; then
            task_msg="data: $pending_task_id"
            task_msg_from_pending=true
            rm -f "$PENDING_TASK_ID_FILE"
            echo "Using pending task command: task_id=$pending_task_id"
        fi
    fi

    # Listen to /task_id topic, wait for new messages
    if [ "$task_msg_from_pending" = false ]; then
        echo "Listening to /task_id messages..."

        # Use rostopic echo to get latest message (10 second timeout)
        task_msg=$(timeout 10 rostopic echo -n 1 /task_id 2>/dev/null)
        rostopic_status=$?
    else
        rostopic_status=0
    fi
    
    if [ $rostopic_status -eq 0 ] && [ ! -z "$task_msg" ]; then
        # Calculate message hash
        if [ "$task_msg_from_pending" = true ]; then
            current_hash="pending_$(date +%s.%N)"
        else
            current_hash=$(echo "$task_msg" | md5sum | cut -d' ' -f1)
        fi
        
        # Only process when there is an actual new task message.
        if [ "$task_msg_from_pending" = false ] && [ "$current_hash" = "$last_message_hash" ]; then
            echo "No new /task_id message. Keeping current task: $last_task_id"
            sleep 0.2
            continue
        fi

        last_message_hash="$current_hash"
            
            # Extract task_id value
            current_task_id=$(echo "$task_msg" | grep "data:" | awk '{print $2}' | tr -d '\n\r')
            
            if [ ! -z "$current_task_id" ]; then
                echo ""
                echo "=== Received new task command: task_id=$current_task_id ==="
                echo "Last task ID: $last_task_id"
                
                # Check if task switching is needed
                if [ ! -z "$last_task_id" ] && [ "$last_task_id" != "$current_task_id" ]; then
                    echo "Detected task switch: $last_task_id -> $current_task_id"
                    
                    # Land command does not switch task state; skip stop when switching from track to land
                    if [ "$last_task_id" = "2" ] && [ "$current_task_id" = "3" ]; then
                        echo "Switching from track to land command, skipping stop script"
                    else
                        # Execute stop script based on last task ID
                        if [ "$last_task_id" = "1" ]; then
                            # Cleanup old stop script PID file
                            rm -f "$STOP_SCRIPT_PID_FILE"
                            
                            echo "Executing stop_ego.sh..."
                            if [ -f "$SCRIPT_DIR/stop_ego.sh" ]; then
                                "$SCRIPT_DIR/stop_ego.sh" &
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
                            if [ -f "$SCRIPT_DIR/stop_track.sh" ]; then
                                "$SCRIPT_DIR/stop_track.sh" &
                                STOP_SCRIPT_PID=$!
                                echo $STOP_SCRIPT_PID > "$STOP_SCRIPT_PID_FILE"
                                echo "  stop_track.sh started (PID: $STOP_SCRIPT_PID)"
                                
                                # Wait for stop script to complete
                                wait $STOP_SCRIPT_PID 2>/dev/null
                                echo "stop_track.sh execution completed"
                            else
                                echo "Warning: stop_track.sh script not found"
                            fi
                            
                        elif [ "$last_task_id" = "4" ]; then
                            # Cleanup old stop script PID file
                            rm -f "$STOP_SCRIPT_PID_FILE"
                            
                            echo "Executing stop_perch.sh..."
                            if [ -f "$SCRIPT_DIR/stop_perch.sh" ]; then
                                "$SCRIPT_DIR/stop_perch.sh" &
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
                            "$SCRIPT_DIR/start_ego.sh" &
                            START_EGO_PID=$!
                            echo $START_EGO_PID > "$START_EGO_PID_FILE"
                            echo "  start_ego.sh started (PID: $START_EGO_PID)"
                        else
                            echo "Already in ego-plan task, skipping startup"
                        fi
                    fi

                    echo "Task 1: waiting for /goal_with_id_from_station_origin and forwarding valid GoalSet..."
                    wait_and_forward_goalset_origin
                    wait_result=$?
                    if [ $wait_result -eq 2 ]; then
                        echo "Task 1 interrupted by a new task command. Switching immediately..."
                        echo "=== Task execution interrupted, continuing to listen... ==="
                        echo ""
                        continue
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
                            start_track
                        else
                            echo "Already in track task, skipping startup"
                        fi
                    fi
                    
                    # Save current task ID
                    echo "$current_task_id" > "$LAST_TASK_ID_FILE"
                    last_task_id="$current_task_id"
                    
                elif [ "$current_task_id" = "3" ]; then
                    echo "Executing task: Send land command"
                    
                    # Execute land command
                    send_land_command
                    
                    # Don't update last_task_id, maintain original task state
                    echo "Note: Land command sent, task status remains unchanged"
                    echo "Currently still in task: $last_task_id"
                    
                elif [ "$current_task_id" = "4" ]; then
                    echo "Executing task: Start perch"
                    
                    if [ "$first_execution" = true ]; then
                        echo "First execution, directly starting perch..."
                        start_perch
                        first_execution=false
                        rm -f "$FIRST_EXECUTION_FLAG"
                    else
                        echo "Not first execution, checking if startup needed..."
                        if [ "$last_task_id" != "4" ]; then
                            echo "Switching from other task, starting perch..."
                            # Start start_perch.sh and record PID
                            rm -f "$START_PERCH_PID_FILE"
                            "$SCRIPT_DIR/start_perch.sh" &
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
        # No new task_id: keep current task behavior.
        if [ "$last_task_id" = "1" ]; then
            echo "No new task_id, continue task 1 goal forwarding..."
            wait_and_forward_goalset_origin
            wait_result=$?
            if [ $wait_result -eq 2 ]; then
                echo "Task 1 interrupted by a new task command. Switching immediately..."
                echo "=== Task execution interrupted, continuing to listen... ==="
                echo ""
                continue
            fi
        elif [ "$last_task_id" = "2" ]; then
            ensure_track_process_running
            sleep 0.5
        else
            sleep 0.5
        fi
    fi
done

# Script normally won't reach here
cleanup
