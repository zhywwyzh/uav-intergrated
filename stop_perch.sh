#!/bin/bash
# Stop perching related nodes and save position information

echo "=== Stopping Perching Related Nodes and Saving Position Information ==="
echo ""

# Temporary file path
POSITION_TMP_FILE="/tmp/drone_position.tmp"

# Store all rostopic pub process PIDs
ROSTOPIC_PIDS=()

# Cleanup function - called when script exits or interrupts
cleanup() {
    echo ""
    echo "=== Received interrupt signal, starting cleanup... ==="
    
    # Terminate all rostopic pub background processes
    if [ ${#ROSTOPIC_PIDS[@]} -gt 0 ]; then
        echo "  Terminating all rostopic pub background processes..."
        for pid in "${ROSTOPIC_PIDS[@]}"; do
            if [ ! -z "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                kill -TERM "$pid" 2>/dev/null >/dev/null 2>&1
            fi
        done
        ROSTOPIC_PIDS=()
        pkill -f "rostopic pub" 2>/dev/null
    fi
    
    echo "=== Cleanup completed ==="
}

# Register signal handlers
trap cleanup SIGINT SIGTERM SIGQUIT

# 1. Save current drone position to temporary file
echo "1. Getting and saving current drone position information..."

# Set default values
INIT_X="0.0"
INIT_Y="0.0"
INIT_Z="2.0"
INIT_YAW="0.0"

# Try to get position information from odom topic
ODOM_TOPICS=("/odom")

position_found=false

for odom_topic in "${ODOM_TOPICS[@]}"; do
    if rostopic list | grep -q "^$odom_topic$"; then
        echo "  Getting position and orientation information from topic $odom_topic..."
        
        # Try to get latest odom message
        odom_msg=$(timeout 1 rostopic echo -n 1 $odom_topic 2>/dev/null | head -30)
        
        if [ ! -z "$odom_msg" ]; then
            # Extract position information
            pos_x=$(echo "$odom_msg" | grep "position:" -A 3 | grep "x:" | awk '{print $2}')
            pos_y=$(echo "$odom_msg" | grep "position:" -A 3 | grep "y:" | awk '{print $2}')
            pos_z=$(echo "$odom_msg" | grep "position:" -A 3 | grep "z:" | awk '{print $2}')
            
            # Extract orientation information (quaternion)
            orient_x=$(echo "$odom_msg" | grep "orientation:" -A 4 | grep "x:" | awk '{print $2}')
            orient_y=$(echo "$odom_msg" | grep "orientation:" -A 4 | grep "y:" | awk '{print $2}')
            orient_z=$(echo "$odom_msg" | grep "orientation:" -A 4 | grep "z:" | awk '{print $2}')
            orient_w=$(echo "$odom_msg" | grep "orientation:" -A 4 | grep "w:" | awk '{print $2}')
            
            # Handle scientific notation representation
            pos_x=$(echo "$pos_x" | sed 's/e/*10^/g; s/E/*10^/g')
            pos_y=$(echo "$pos_y" | sed 's/e/*10^/g; s/E/*10^/g')
            pos_z=$(echo "$pos_z" | sed 's/e/*10^/g; s/E/*10^/g')
            orient_x=$(echo "$orient_x" | sed 's/e/*10^/g; s/E/*10^/g')
            orient_y=$(echo "$orient_y" | sed 's/e/*10^/g; s/E/*10^/g')
            orient_z=$(echo "$orient_z" | sed 's/e/*10^/g; s/E/*10^/g')
            orient_w=$(echo "$orient_w" | sed 's/e/*10^/g; s/E/*10^/g')
            
            # Use awk to calculate actual values
            pos_x=$(echo "$pos_x" | awk '{print $1}')
            pos_y=$(echo "$pos_y" | awk '{print $1}')
            pos_z=$(echo "$pos_z" | awk '{print $1}')
            orient_x=$(echo "$orient_x" | awk '{print $1}')
            orient_y=$(echo "$orient_y" | awk '{print $1}')
            orient_z=$(echo "$orient_z" | awk '{print $1}')
            orient_w=$(echo "$orient_w" | awk '{print $1}')
            
            if [ ! -z "$pos_x" ] && [ ! -z "$pos_y" ] && [ ! -z "$pos_z" ]; then
                INIT_X="$pos_x"
                INIT_Y="$pos_y"
                INIT_Z="$pos_z"
                position_found=true
                
                # If orientation information exists, calculate yaw angle
                if [ ! -z "$orient_x" ] && [ ! -z "$orient_y" ] && [ ! -z "$orient_z" ] && [ ! -z "$orient_w" ]; then
                    # Calculate yaw angle
                    INIT_YAW=$(awk -v x="$orient_x" -v y="$orient_y" -v z="$orient_z" -v w="$orient_w" 'BEGIN {
                        norm = sqrt(x*x + y*y + z*z + w*w)
                        if (norm > 0) {
                            x = x / norm; y = y / norm; z = z / norm; w = w / norm
                        } else {
                            x = 0; y = 0; z = 0; w = 1
                        }
                        siny_cosp = 2.0 * (w * z + x * y)
                        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
                        yaw = atan2(siny_cosp, cosy_cosp)
                        printf "%.6f", yaw
                    }' 2>/dev/null)
                    
                    if [ -z "$INIT_YAW" ] || [ "$INIT_YAW" = "nan" ] || [ "$INIT_YAW" = "inf" ]; then
                        INIT_YAW="0.0"
                    fi
                fi
                break
            fi
        fi
    fi
done

if [ "$position_found" = false ]; then
    echo "  ⚠ Unable to get drone position information, using default position"
fi

# Save to temporary file
echo "  Saving position parameters to temporary file $POSITION_TMP_FILE..."
cat > "$POSITION_TMP_FILE" << EOF
# Generation time: $(date)
# Format: parameter_name=value

# Position parameters
INIT_X=$INIT_X
INIT_Y=$INIT_Y
INIT_Z=$INIT_Z

# Orientation parameter (yaw angle, unit: radians)
INIT_YAW=$INIT_YAW

# Quaternion representation (for ROS topics)
ORIENT_Z=$(echo "s($INIT_YAW/2)" | bc -l 2>/dev/null || echo "0.0")
ORIENT_W=$(echo "c($INIT_YAW/2)" | bc -l 2>/dev/null || echo "1.0")
EOF

echo "  Position parameters saved: x=$INIT_X, y=$INIT_Y, z=$INIT_Z, yaw=$INIT_YAW rad"
echo "  Temporary file: $POSITION_TMP_FILE"

echo ""
echo "2. Starting to terminate perching related nodes..."

# 2. Define nodes to terminate (based on perching.launch)
TARGET_NODES=(
    "/odom_visualization"
    "/odom_visualization_plate"
    "/manager"
    "/planning"
)

# 3. Precisely terminate nodes
echo "Processing perching nodes:"
for node in "${TARGET_NODES[@]}"; do
    echo "Processing: $node"
    if rosnode ping "$node" -c 1 &>/dev/null 2>&1; then
        echo "  ✓ Found and terminating..."
        rosnode kill "$node" 2>/dev/null
    else
        echo "  ✗ Node not running"
    fi
done

# Terminate all nodelet processes
echo ""
echo "Processing nodelet related processes:"
if pgrep -f "nodelet" >/dev/null; then
    echo "  ✓ Found and terminating nodelet processes..."
    pkill -f "nodelet" 2>/dev/null
else
    echo "  ✗ No nodelet processes"
fi

echo ""
echo "3. Clearing specified visualization topics..."

# Clear three specified topics
echo "  Clearing /odom_visualization_plate/polygon (visualization_msgs/Marker)..."
(
rostopic pub -1 /odom_visualization_plate/polygon visualization_msgs/Marker "
header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'world'
ns: 'polygon'
id: 0
action: 3
pose:
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
scale: {x: 1.0, y: 1.0, z: 1.0}
color: {r: 0.0, g: 0.0, b: 0.0, a: 0.0}
lifetime: {secs: 0, nsecs: 0}
frame_locked: false
points: []
colors: []
" &
pid1=$!
ROSTOPIC_PIDS+=($pid1)

echo "  Clearing /planning/traj (nav_msgs/Path)..."
rostopic pub -1 /planning/traj nav_msgs/Path "
header:
  seq: 0
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: 'world'
poses: []
" &
pid2=$!
ROSTOPIC_PIDS+=($pid2)

echo "  Clearing /odom_visualization/fov_visual (visualization_msgs/MarkerArray)..."
rostopic pub -1 /odom_visualization/fov_visual visualization_msgs/MarkerArray "
markers:
- header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: 'world'
  ns: 'fov'
  id: 0
  action: 3
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  scale: {x: 1.0, y: 1.0, z: 1.0}
  color: {r: 0.0, g: 0.0, b: 0.0, a: 0.0}
  lifetime: {secs: 0, nsecs: 0}
  frame_locked: false
" &
pid3=$!
ROSTOPIC_PIDS+=($pid3)

# Wait for all rostopic pub commands to complete
wait $pid1 $pid2 $pid3 2>/dev/null

# Remove completed processes from PID array
ROSTOPIC_PIDS=()
) >/dev/null 2>&1

echo ""
echo "=== PERCH Stop Completed ==="
