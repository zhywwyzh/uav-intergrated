#!/bin/bash
# build_ws.sh - one-click compile selected ROS workspaces

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ROS_SETUP="/opt/ros/noetic/setup.bash"
CLEAN_BUILD=0
JOBS=""

declare -A WS_PATHS
WS_PATHS[ego]="$PROJECT_ROOT/ego-planner"
WS_PATHS[track]="$PROJECT_ROOT/Elastic-Tracker"
WS_PATHS[perch]="$PROJECT_ROOT/Fast-Perching"

WORKSPACE_ORDER=(ego track perch)

usage() {
    cat <<'EOF'
Usage:
  bash scripts/build_ws.sh
  bash scripts/build_ws.sh [all|ego|track|perch ...] [--clean] [--jobs N]

Examples:
  bash scripts/build_ws.sh
  bash scripts/build_ws.sh all
  bash scripts/build_ws.sh ego perch --jobs 8
  bash scripts/build_ws.sh track --clean

Notes:
  - No positional args: interactive selection mode.
  - --clean means remove build/devel before compiling selected workspaces.
EOF
}

append_unique() {
    local item="$1"
    local x
    for x in "${SELECTED[@]:-}"; do
        [ "$x" = "$item" ] && return 0
    done
    SELECTED+=("$item")
}

add_all() {
    local ws
    for ws in "${WORKSPACE_ORDER[@]}"; do
        append_unique "$ws"
    done
}

select_interactive() {
    echo "Select ROS workspaces to compile:"
    echo "  1) ego-planner"
    echo "  2) Elastic-Tracker"
    echo "  3) Fast-Perching"
    echo "  a) all"
    echo "  q) quit"
    echo ""
    read -r -p "Input (e.g. 1 3 / 1,2 / a): " user_input

    user_input="$(echo "$user_input" | tr ',' ' ')"
    for token in $user_input; do
        case "$token" in
            1|ego|planner|ego-planner)
                append_unique ego
                ;;
            2|track|elastic|elastic-tracker)
                append_unique track
                ;;
            3|perch|fast|fast-perching)
                append_unique perch
                ;;
            a|all)
                add_all
                ;;
            q|quit|exit)
                echo "Exit."
                exit 0
                ;;
            *)
                echo "Warning: ignore unknown option '$token'"
                ;;
        esac
    done
}

parse_args() {
    SELECTED=()
    while [ $# -gt 0 ]; do
        case "$1" in
            -h|--help)
                usage
                exit 0
                ;;
            --clean)
                CLEAN_BUILD=1
                shift
                ;;
            -j|--jobs)
                if [ $# -lt 2 ]; then
                    echo "Error: $1 requires a value"
                    exit 2
                fi
                JOBS="$2"
                shift 2
                ;;
            all)
                add_all
                shift
                ;;
            ego|planner|ego-planner)
                append_unique ego
                shift
                ;;
            track|elastic|elastic-tracker)
                append_unique track
                shift
                ;;
            perch|fast|fast-perching)
                append_unique perch
                shift
                ;;
            *)
                echo "Error: unknown argument '$1'"
                usage
                exit 2
                ;;
        esac
    done

    if [ "${#SELECTED[@]}" -eq 0 ]; then
        select_interactive
    fi

    if [ "${#SELECTED[@]}" -eq 0 ]; then
        echo "Error: no workspace selected"
        exit 2
    fi
}

build_one() {
    local ws_key="$1"
    local ws_path="${WS_PATHS[$ws_key]}"
    local jobs_arg=()

    if [ -n "$JOBS" ]; then
        jobs_arg=(-j "$JOBS")
    fi

    if [ ! -d "$ws_path" ]; then
        echo "[FAIL] $ws_key: workspace directory not found: $ws_path"
        return 1
    fi

    echo ""
    echo "=== Building $ws_key ==="
    echo "Path: $ws_path"

    (
        set -e
        cd "$ws_path"

        if [ -f "$ROS_SETUP" ]; then
            # shellcheck disable=SC1090
            source "$ROS_SETUP"
        fi

        if [ "$CLEAN_BUILD" -eq 1 ]; then
            echo "Cleaning build/devel in $ws_path"
            rm -rf build devel
        fi

        catkin_make "${jobs_arg[@]}"
    )
}

main() {
    parse_args "$@"

    echo "Selected workspaces: ${SELECTED[*]}"
    echo "Clean build: $CLEAN_BUILD"
    [ -n "$JOBS" ] && echo "Jobs: $JOBS"

    local ok_list=()
    local fail_list=()
    local ws

    for ws in "${SELECTED[@]}"; do
        if build_one "$ws"; then
            ok_list+=("$ws")
        else
            fail_list+=("$ws")
        fi
    done

    echo ""
    echo "=== Build Summary ==="
    [ "${#ok_list[@]}" -gt 0 ] && echo "Success: ${ok_list[*]}"
    [ "${#fail_list[@]}" -gt 0 ] && echo "Failed : ${fail_list[*]}"

    if [ "${#fail_list[@]}" -gt 0 ]; then
        exit 1
    fi
}

main "$@"
