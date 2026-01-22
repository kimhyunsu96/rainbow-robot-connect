#!/bin/bash

# RB Test Package Manager Script v2.0
# Enhanced with chunked motion executor support for RB10-1300E
# Fixes >50 waypoint limitation issue

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SMART_WS_DIR:-$THIS_DIR}"
PACKAGE_NAME="rb_test"

# Function to display main menu
show_menu() {
    clear
    echo -e "${CYAN}╔═══════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║      RB Test Package Manager v2.1                 ║${NC}"
    echo -e "${CYAN}║      (30-Waypoint Chunking for Stability)         ║${NC}"
    echo -e "${CYAN}╚═══════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${GREEN}=== Build & Run ===${NC}"
    echo -e "${GREEN}1)${NC} Build Package"
    echo -e "${GREEN}2)${NC} Run All Nodes (Standard)"
    echo -e "${GREEN}3)${NC} Run All Nodes (Chunked Mode) ${YELLOW}[NEW]${NC}"
    echo -e "${GREEN}4)${NC} Run Modbus Bridge Only"
    echo -e "${GREEN}5)${NC} Run Motion GUI Only (Standard)"
    echo -e "${GREEN}6)${NC} Run Motion GUI (Chunked) ${YELLOW}[NEW]${NC}"
    echo ""
    echo -e "${BLUE}=== Motion Tools ===${NC}"
    echo -e "${BLUE}7)${NC} Split YAML Motion File ${YELLOW}[NEW]${NC}"
    echo -e "${BLUE}8)${NC} Test Chunked Executor ${YELLOW}[NEW]${NC}"
    echo ""
    echo -e "${MAGENTA}=== Monitor & Debug ===${NC}"
    echo -e "${MAGENTA}9)${NC} Monitor Topics"
    echo -e "${MAGENTA}10)${NC} View Node Info"
    echo -e "${MAGENTA}11)${NC} System Check"
    echo ""
    echo -e "${CYAN}=== Utilities ===${NC}"
    echo -e "${CYAN}12)${NC} Clean Build"
    echo -e "${CYAN}13)${NC} Install Dependencies"
    echo -e "${CYAN}14)${NC} Quick Start (Build + Run Chunked)"
    echo ""
    echo -e "${RED}0)${NC} Exit"
    echo ""
    echo -n "Select option: "
}

# Function to check ROS2 environment
check_ros2() {
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}ROS2 is not installed or not sourced${NC}"
        echo "Attempting to source ROS2..."
        source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null || {
            echo -e "${RED}Failed to source ROS2${NC}"
            return 1
        }
    fi
    echo -e "${GREEN}✓ ROS2 environment OK${NC}"
    return 0
}

# Function to build package
build_package() {
    echo -e "${YELLOW}Building $PACKAGE_NAME package with chunking support...${NC}"
    cd "$WORKSPACE_DIR"
    
    # Source ROS2
    source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null
    
    # Clean and build
    colcon build --packages-select $PACKAGE_NAME --symlink-install
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Build successful!${NC}"
        echo -e "${GREEN}✓ New modules available:${NC}"
        echo "  - motion_executor_chunked"
        echo "  - motion_yaml_splitter"
        source "$WORKSPACE_DIR/install/setup.bash"
    else
        echo -e "${RED}✗ Build failed!${NC}"
        return 1
    fi
}

# Function to run all nodes (standard mode)
run_all_nodes() {
    echo -e "${YELLOW}Starting all nodes (Standard Mode)...${NC}"
    
    # Get parameters
    echo -n "Robot IP [192.168.1.13]: "
    read robot_ip
    robot_ip=${robot_ip:-192.168.1.13}
    
    echo -n "DI Address [12]: "
    read di_address
    di_address=${di_address:-12}
    echo -n "Unity PC IP to allow (empty=all) []: "
    read unity_allow
    echo -n "P_S topic [/p_s]: "
    read ps_topic
    ps_topic=${ps_topic:-/p_s}
    
    cd "$WORKSPACE_DIR"
    source "$WORKSPACE_DIR/install/setup.bash"
    
    echo -e "${GREEN}Launching with:${NC}"
    echo "  Robot IP: $robot_ip"
    echo "  DI Address: $di_address"
    echo "  Unity allow: ${unity_allow:-<any>}"
    echo "  P_S topic: ${ps_topic}"
    
    ros2 launch $PACKAGE_NAME rb_test_all.launch.py \
        robot_ip:=$robot_ip \
        di_address:=$di_address \
        unity_listen:=0.0.0.0 \
        unity_port:=10001 \
        unity_allow:="$unity_allow" \
        unity_robot_ip:="$robot_ip" \
        ps_value_topic:="$ps_topic"
}

# Function to run all nodes with chunking support
run_all_nodes_chunked() {
    echo -e "${YELLOW}Starting all nodes (Chunked Mode for >50 waypoints)...${NC}"
    echo -e "${CYAN}This mode automatically splits large motions into 30-waypoint chunks${NC}"
    
    # Get parameters
    echo -n "Robot IP [192.168.1.13]: "
    read robot_ip
    robot_ip=${robot_ip:-192.168.1.13}
    
    echo -n "DI Address [12]: "
    read di_address
    di_address=${di_address:-12}
    
    echo -n "Unity PC IP to allow (empty=all) []: "
    read unity_allow
    
    echo -n "Chunk size (max waypoints per chunk) [30]: "
    read chunk_size
    chunk_size=${chunk_size:-30}
    
    echo -n "Delay between chunks (ms) [200]: "
    read chunk_delay
    chunk_delay=${chunk_delay:-200}
    echo -n "P_S topic [/p_s]: "
    read ps_topic
    ps_topic=${ps_topic:-/p_s}
    
    cd "$WORKSPACE_DIR"
    source "$WORKSPACE_DIR/install/setup.bash"
    
    echo -e "${GREEN}Launching with chunking support:${NC}"
    echo "  Robot IP: $robot_ip"
    echo "  DI Address: $di_address"
    echo "  Chunk Size: $chunk_size waypoints"
    echo "  Chunk Delay: $chunk_delay ms"
    echo "  P_S topic: ${ps_topic}"
    
    # Launch both nodes with chunking parameters
    ros2 launch $PACKAGE_NAME rb_test_all.launch.py \
        robot_ip:=$robot_ip \
        di_address:=$di_address \
        chunk_size:=$chunk_size \
        chunk_delay_ms:=$chunk_delay \
        unity_listen:=0.0.0.0 \
        unity_port:=10001 \
        unity_allow:="$unity_allow" \
        unity_robot_ip:="$robot_ip" \
        ps_value_topic:="$ps_topic"
}

# Function to run Motion GUI with chunking
run_motion_gui_chunked() {
    echo -e "${YELLOW}Starting Motion GUI Runner (Chunked Mode)...${NC}"
    
    echo -n "Unity PC IP to allow (empty=all) []: "
    read unity_allow
    
    echo -n "Chunk size (max waypoints per chunk) [30]: "
    read chunk_size
    chunk_size=${chunk_size:-30}
    
    echo -n "Delay between chunks (ms) [200]: "
    read chunk_delay
    chunk_delay=${chunk_delay:-200}
    echo -n "P_S topic [/p_s]: "
    read ps_topic
    ps_topic=${ps_topic:-/p_s}
    
    cd "$WORKSPACE_DIR"
    source "$WORKSPACE_DIR/install/setup.bash"
    
    echo -e "${GREEN}Launching Motion GUI with chunking:${NC}"
    echo "  Chunk Size: $chunk_size waypoints"
    echo "  Chunk Delay: $chunk_delay ms"
    echo "  P_S topic: ${ps_topic}"
    
    ros2 launch $PACKAGE_NAME motion_gui_runner_chunked.launch.py \
        chunk_size:=$chunk_size \
        chunk_delay_ms:=$chunk_delay \
        unity_listen:=0.0.0.0 \
        unity_port:=10001 \
        unity_allow:="$unity_allow" \
        ps_value_topic:="$ps_topic"
}

# Function to split YAML file
split_yaml_file() {
    echo -e "${YELLOW}YAML Motion File Splitter${NC}"
    echo -e "${CYAN}Split large motion files into 30-waypoint chunks${NC}"
    echo ""
    
    echo -n "Enter YAML file path: "
    read yaml_file
    
    if [ ! -f "$yaml_file" ]; then
        echo -e "${RED}File not found: $yaml_file${NC}"
        echo "Press Enter to continue..."
        read
        return
    fi
    
    echo -n "Max waypoints per chunk [30]: "
    read max_waypoints
    max_waypoints=${max_waypoints:-30}
    
    echo -n "Create sequence file? (y/n) [y]: "
    read create_seq
    create_seq=${create_seq:-y}
    
    cd "$WORKSPACE_DIR"
    source "$WORKSPACE_DIR/install/setup.bash"
    
    cmd="ros2 run $PACKAGE_NAME motion_yaml_splitter $yaml_file -m $max_waypoints"
    
    if [ "$create_seq" = "y" ]; then
        cmd="$cmd -s"
    fi
    
    echo -e "${GREEN}Running: $cmd${NC}"
    eval $cmd
    
    echo ""
    echo "Press Enter to continue..."
    read
}

# Function to test chunked executor
test_chunked_executor() {
    echo -e "${YELLOW}Test Chunked Motion Executor${NC}"
    echo ""
    
    echo -n "Enter motion YAML file path: "
    read yaml_file
    
    if [ ! -f "$yaml_file" ]; then
        echo -e "${RED}File not found: $yaml_file${NC}"
        echo "Press Enter to continue..."
        read
        return
    fi
    
    echo -n "Robot IP [192.168.1.13]: "
    read robot_ip
    robot_ip=${robot_ip:-192.168.1.13}
    
    echo -n "Chunk size [50]: "
    read chunk_size
    chunk_size=${chunk_size:-50}
    
    cd "$WORKSPACE_DIR"
    source "$WORKSPACE_DIR/install/setup.bash"
    
    echo -e "${GREEN}Testing chunked executor with:${NC}"
    echo "  File: $yaml_file"
    echo "  Robot IP: $robot_ip"
    echo "  Chunk Size: $chunk_size"
    
    ros2 run $PACKAGE_NAME motion_executor_chunked \
        --ros-args \
        -p motion_file:="$yaml_file" \
        -p robot_ip:="$robot_ip" \
        -p chunk_size:=$chunk_size \
        -p auto_load:=true
}

# Function to monitor topics
monitor_topics() {
    echo -e "${CYAN}Available ROS2 topics:${NC}"
    ros2 topic list
    echo ""
    echo -n "Enter topic to monitor (or press Enter to go back): "
    read topic
    
    if [ ! -z "$topic" ]; then
        echo -e "${YELLOW}Monitoring $topic (Press Ctrl+C to stop)${NC}"
        ros2 topic echo $topic
    fi
}

# Function to view node info
view_node_info() {
    echo -e "${CYAN}Active ROS2 nodes:${NC}"
    ros2 node list
    echo ""
    echo -n "Enter node name for info (or press Enter to go back): "
    read node
    
    if [ ! -z "$node" ]; then
        ros2 node info $node
        echo ""
        echo "Press Enter to continue..."
        read
    fi
}

# Function to clean build
clean_build() {
    echo -e "${YELLOW}Cleaning build artifacts...${NC}"
    cd "$WORKSPACE_DIR"
    rm -rf build/$PACKAGE_NAME install/$PACKAGE_NAME log/$PACKAGE_NAME
    echo -e "${GREEN}✓ Clean complete${NC}"
}

# Function to install dependencies
install_deps() {
    echo -e "${YELLOW}Installing dependencies...${NC}"
    
    # Python packages
    pip3 install --upgrade pymodbus PyQt5 multipledispatch transforms3d numpy pyyaml
    
    # ROS2 packages
    sudo apt-get update
    sudo apt-get install -y ros-humble-rclpy ros-humble-std-msgs python3-colcon-common-extensions
    
    echo -e "${GREEN}✓ Dependencies installed${NC}"
}

# Function to check system
system_check() {
    echo -e "${CYAN}System Check Report${NC}"
    echo -e "${CYAN}═══════════════════${NC}"
    
    # Check ROS2
    echo -n "ROS2 Environment: "
    if check_ros2 > /dev/null 2>&1; then
        echo -e "${GREEN}✓${NC}"
    else
        echo -e "${RED}✗${NC}"
    fi
    
    # Check workspace
    echo -n "Workspace exists: "
    if [ -d "$WORKSPACE_DIR" ]; then
        echo -e "${GREEN}✓${NC}"
    else
        echo -e "${RED}✗${NC}"
    fi
    
    # Check package
    echo -n "Package source exists: "
    if [ -d "$WORKSPACE_DIR/src/$PACKAGE_NAME" ]; then
        echo -e "${GREEN}✓${NC}"
    else
        echo -e "${RED}✗${NC}"
    fi
    
    # Check build
    echo -n "Package built: "
    if [ -d "$WORKSPACE_DIR/install/$PACKAGE_NAME" ]; then
        echo -e "${GREEN}✓${NC}"
    else
        echo -e "${RED}✗${NC}"
    fi
    
    # Check new modules
    echo ""
    echo -e "${CYAN}New Chunking Modules:${NC}"
    for module in motion_executor_chunked motion_yaml_splitter; do
        echo -n "  $module: "
        if [ -f "$WORKSPACE_DIR/src/$PACKAGE_NAME/rb_test/${module}.py" ]; then
            echo -e "${GREEN}✓${NC}"
        else
            echo -e "${RED}✗${NC}"
        fi
    done
    
    # Check Python modules
    echo ""
    echo -e "${CYAN}Python Dependencies:${NC}"
    for module in pymodbus PyQt5 multipledispatch transforms3d numpy yaml; do
        echo -n "  $module: "
        if python3 -c "import $module" 2>/dev/null; then
            echo -e "${GREEN}✓${NC}"
        else
            echo -e "${RED}✗${NC}"
        fi
    done
    
    echo ""
    echo "Press Enter to continue..."
    read
}

# Function for quick start with chunking
quick_start() {
    echo -e "${MAGENTA}Quick Start - Building and Running with Chunking Support${NC}"
    echo -e "${MAGENTA}════════════════════════════════════════════════════════${NC}"
    
    # Build
    build_package
    if [ $? -ne 0 ]; then
        echo -e "${RED}Build failed. Aborting quick start.${NC}"
        return 1
    fi
    
    # Small delay
    sleep 2
    
    # Run with chunking
    run_all_nodes_chunked
}

# Main loop
while true; do
    show_menu
    read -r choice
    
    case $choice in
        1)
            build_package
            echo "Press Enter to continue..."
            read
            ;;
        2)
            run_all_nodes
            ;;
        3)
            run_all_nodes_chunked
            ;;
        4)
            cd "$WORKSPACE_DIR"
            source "$WORKSPACE_DIR/install/setup.bash"
            echo -n "Robot IP [192.168.1.13]: "
            read robot_ip
            robot_ip=${robot_ip:-192.168.1.13}
            ros2 launch $PACKAGE_NAME rb_modbus_bridge.launch.py robot_ip:=$robot_ip
            ;;
        5)
            cd "$WORKSPACE_DIR"
            source "$WORKSPACE_DIR/install/setup.bash"
            echo -n "P_S topic [/p_s]: "
            read ps_topic
            ps_topic=${ps_topic:-/p_s}
            echo -e "${GREEN}Launching Motion GUI:${NC}"
            echo "  P_S topic: ${ps_topic}"
            ros2 launch $PACKAGE_NAME motion_gui_runner.launch.py ps_value_topic:="$ps_topic"
            ;;
        6)
            run_motion_gui_chunked
            ;;
        7)
            split_yaml_file
            ;;
        8)
            test_chunked_executor
            ;;
        9)
            monitor_topics
            ;;
        10)
            view_node_info
            ;;
        11)
            system_check
            ;;
        12)
            clean_build
            echo "Press Enter to continue..."
            read
            ;;
        13)
            install_deps
            echo "Press Enter to continue..."
            read
            ;;
        14)
            quick_start
            ;;
        0)
            echo -e "${GREEN}Exiting...${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid option${NC}"
            sleep 1
            ;;
    esac
done
