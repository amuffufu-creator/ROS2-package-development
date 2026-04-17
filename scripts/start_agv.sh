#!/bin/bash
# ============================================
# AGV Navigation - One Click Launcher
# ============================================
# Double-click this file to start the full
# navigation stack with RViz.
#
# Usage:
#   1. Set ESP32_IP below
#   2. Set MAP_FILE to your saved map
#   3. Double-click or run: ./start_agv.sh
# ============================================

ESP32_IP="192.168.100.25"
MAP_FILE="$HOME/ros2_ws/maps/my_map.yaml"

# ─── Source ROS 2 ───
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

# ─── Check map file exists ───
if [ ! -f "$MAP_FILE" ]; then
    echo "ERROR: Map file not found: $MAP_FILE"
    echo "Please run SLAM first to create a map."
    read -p "Press Enter to exit..."
    exit 1
fi

# ─── Launch ───
echo "============================================"
echo "  AGV Navigation System"
echo "============================================"
echo "  ESP32 IP : $ESP32_IP"
echo "  Map      : $MAP_FILE"
echo "============================================"
echo ""
echo "  Steps:"
echo "    1. Wait for all nodes to start"
echo "    2. In RViz: click '2D Pose Estimate'"
echo "       and set robot position on map"
echo "    3. Click '2D Goal Pose' to send robot"
echo "       to a destination"
echo ""
echo "  Press Ctrl+C to stop everything"
echo "============================================"
echo ""

ros2 launch agv_base_controller agv_navigation.launch.py \
    esp32_ip:="$ESP32_IP" \
    map:="$MAP_FILE"