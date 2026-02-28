#!/bin/bash
# task_pub.sh - Simplified ROS1 Task Publisher Script (only publishes task_id: 1-4)

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if roscore is running
check_ros() {
    if ! rostopic list > /dev/null 2>&1; then
        echo -e "${RED}Error: roscore not running!${NC}"
        echo "Please run: roscore"
        exit 1
    fi
    echo -e "${GREEN}✓ ROS core running normally${NC}"
}

# Publish task_id
publish_task() {
    local task_id=$1
    local task_name=""
    
    case $task_id in
        1) task_name="ego-plan";;
        2) task_name="track";;
        3) task_name="land";;
        4) task_name="perch";;
    esac
    
    echo -e "${YELLOW}Publishing task_id=$task_id [$task_name]...${NC}"
    
    rostopic pub -1 /task_id std_msgs/Int32 "data: $task_id"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Successfully published task_id=$task_id${NC}"
        return 0
    else
        echo -e "${RED}✗ Publication failed${NC}"
        return 1
    fi
}

# Display menu
show_menu() {
    clear
    echo -e "${BLUE}==========================================${NC}"
    echo -e "${YELLOW}           ROS Task Publisher${NC}"
    echo -e "${BLUE}==========================================${NC}"
    echo -e "Select task_id to publish:"
    echo -e "  ${GREEN}1${NC} - Publish task_id=1 ${YELLOW}[ego-plan]${NC}"
    echo -e "  ${GREEN}2${NC} - Publish task_id=2 ${YELLOW}[track]${NC}"
    echo -e "  ${GREEN}3${NC} - Publish task_id=3 ${YELLOW}[land]${NC}"
    echo -e "  ${GREEN}4${NC} - Publish task_id=4 ${YELLOW}[perch]${NC}"
    echo -e "  ${RED}q${NC} - Exit program"
    echo -e "${BLUE}==========================================${NC}"
}

# Main function
main() {
    # Check ROS environment
    check_ros
    
    while true; do
        show_menu
        
        # Read user input
        read -p "Select [1-4/q]: " choice
        
        case $choice in
            1)
                if publish_task 1; then
                    echo -e "${GREEN}✓ ego-plan task published${NC}"
                fi
                ;;
            2)
                if publish_task 2; then
                    echo -e "${GREEN}✓ track task published${NC}"
                fi
                ;;
            3)
                if publish_task 3; then
                    echo -e "${GREEN}✓ land task published${NC}"
                fi
                ;;
            4)
                if publish_task 4; then
                    echo -e "${GREEN}✓ perch task published${NC}"
                fi
                ;;
            q|Q)
                echo -e "${BLUE}Exiting program...${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}Invalid selection, please try again!${NC}"
                ;;
        esac
        
        # Wait for key press to continue
        echo ""
        echo -e "${BLUE}------------------------------------------${NC}"
        read -n 1 -s -p "Press any key to return to menu..."
    done
}

# Start main function
main
