#!/bin/bash
# task_pub.sh - interactive trigger menu (direct-trigger mode)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

check_ros() {
    if ! rostopic list >/dev/null 2>&1; then
        echo -e "${RED}Error: roscore not running!${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ ROS core running${NC}"
}

show_menu() {
    clear
    echo -e "${BLUE}==========================================${NC}"
    echo -e "${YELLOW}      Module Trigger Menu (Direct Trigger)${NC}"
    echo -e "${BLUE}==========================================${NC}"
    echo -e "  ${GREEN}1${NC} - Trigger ego"
    echo -e "  ${GREEN}2${NC} - Trigger track (trigger only)"
    echo -e "  ${GREEN}3${NC} - Trigger track land"
    echo -e "  ${GREEN}4${NC} - Trigger perch"
    echo -e "  ${RED}q${NC} - Exit"
    echo -e "${BLUE}==========================================${NC}"
}

main() {
    check_ros
    while true; do
        show_menu
        read -p "Select [1-4/q]: " choice
        case "$choice" in
            1)
                python3 "$PROJECT_ROOT/tools/trigger_ego.py"
                ;;
            2)
                python3 "$PROJECT_ROOT/tools/trigger_track.py" --no-fake-inputs --duration 2
                ;;
            3)
                python3 "$PROJECT_ROOT/tools/trigger_track_land.py"
                ;;
            4)
                python3 "$PROJECT_ROOT/tools/trigger_perch.py"
                ;;
            q|Q)
                echo -e "${BLUE}Exiting...${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}Invalid selection${NC}"
                ;;
        esac

        echo ""
        echo -e "${BLUE}------------------------------------------${NC}"
        read -n 1 -s -p "Press any key to continue..."
    done
}

main
