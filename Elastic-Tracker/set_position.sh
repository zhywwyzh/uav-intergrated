#!/bin/bash
# quick_start.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TMP_DIR="$PROJECT_ROOT/tmp"
mkdir -p "$TMP_DIR"

# 从文件提取位置和朝向
LOCATION_FILE="$TMP_DIR/drone_final_location.yaml"
YAW_FILE="$TMP_DIR/drone_init_yaw.txt"

# 提取位置
X=$(grep -A 3 "position:" "$LOCATION_FILE" 2>/dev/null | grep "x:" | awk '{print $2}' || echo "0.0")
Y=$(grep -A 3 "position:" "$LOCATION_FILE" 2>/dev/null | grep "y:" | awk '{print $2}' || echo "0.0")
Z=$(grep -A 3 "position:" "$LOCATION_FILE" 2>/dev/null | grep "z:" | awk '{print $2}' || echo "2.0")

# 从专门的文件读取偏航角
INIT_YAW=$(cat "$YAW_FILE" 2>/dev/null | head -1 || echo "1.570796")

echo "启动参数："
echo "  位置: x=$X, y=$Y, z=$Z"
echo "  朝向: yaw=$INIT_YAW rad"

# 启动
source devel/setup.sh
roslaunch planning simulation1.launch \
    init_x_:="$X" \
    init_y_:="$Y" \
    init_z_:="$Z" \
    init_yaw_:="$INIT_YAW"
