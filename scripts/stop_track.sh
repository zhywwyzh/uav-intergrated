#!/bin/bash
# Lightweight wrapper: load env + run Python stop helper.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TMP_DIR="$PROJECT_ROOT/tmp"
mkdir -p "$TMP_DIR"

echo "=== stop_track.sh ==="

# Keep shell responsibilities: ROS env loading.
if [ -f "$PROJECT_ROOT/Elastic-Tracker/devel/setup.sh" ]; then
    # shellcheck disable=SC1091
    source "$PROJECT_ROOT/Elastic-Tracker/devel/setup.sh"
fi

python3 "$PROJECT_ROOT/utils/stop_task_helper.py" \
    --mode track \
    --tmp-dir "$TMP_DIR"
