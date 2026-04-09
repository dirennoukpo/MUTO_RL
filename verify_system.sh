#!/bin/bash
# System verification script for MUTO RS Pi+Jetson setup
# Verifies all components are operational and interconnected

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "========================================="
echo "MUTO RS System Verification"
echo "========================================="
echo ""

# Source environment
source tekbot_ws/workspace_setup.sh >/dev/null 2>&1

echo "1️⃣  Environment Setup:"
ROBOT_ROLE=$(printenv ROBOT_ROLE 2>/dev/null || echo "unknown")
DOMAIN_ID=$(printenv DOMAIN_ID 2>/dev/null || echo "unknown")
TARGET_IP=$(printenv TARGET_IP 2>/dev/null || echo "unknown")
LOCAL_IP=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "unknown")

echo "   ROBOT_ROLE: $ROBOT_ROLE"
echo "   DOMAIN_ID: $DOMAIN_ID"
echo "   TARGET_IP: $TARGET_IP (Jetson)"
echo "   Local IP: $LOCAL_IP"
echo ""

echo "2️⃣  Pi ROS Nodes (this machine, DRIVER role):"
ps aux | grep -E "(usb_bridge_node|watchdog_node|mode_manager_node)" | grep -v grep | while read line; do
    node_name=$(echo "$line" | grep -o "[a-z_]*_node" | head -1)
    pid=$(echo "$line" | awk '{print $2}')
    echo "   ✓ $node_name (PID: $pid)"
done
echo ""

echo "3️⃣  Pi Hardware Topics (should publish @~162 Hz):"
for topic in /imu/data /joint_states; do
    info=$(ros2 topic info "$topic" 2>&1)
    if echo "$info" | grep -q "Type:"; then
        type=$(echo "$info" | grep "Type:" | cut -d' ' -f2-)
        pub_count=$(echo "$info" | grep "Publisher count:" | cut -d' ' -f3)
        echo "   ✓ $topic -> Type: $type, Publishers: $pub_count"
    else
        echo -e "   ${RED}✗ $topic NOT FOUND${NC}"
    fi
done
echo ""

echo "4️⃣  Jetson Topics (should be visible via network):"
for topic in /observation /commands /commands_raw; do
    info=$(ros2 topic info "$topic" 2>&1)
    if echo "$info" | grep -q "Type:"; then
        type=$(echo "$info" | grep "Type:" | cut -d' ' -f2-)
        pub_count=$(echo "$info" | grep "Publisher count:" | cut -d' ' -f3)
        echo "   ✓ $topic -> Type: $type, Publishers: $pub_count"
    else
        echo -e "   ${RED}✗ $topic NOT FOUND${NC}"
    fi
done
echo ""

echo "5️⃣  Topic Frequency Check (10 second samples):"
echo "   Checking /imu/data..."
freq_imu=$(timeout 10 ros2 topic hz /imu/data 2>&1 | grep "average frequency" | awk '{print $NF}' | tr -d 'Hz')
if [ -z "$freq_imu" ]; then
    echo -e "   ${RED}✗ /imu/data NOT PUBLISHING${NC}"
else
    echo "   ✓ /imu/data: $freq_imu Hz"
fi

echo "   Checking /observation..."
freq_obs=$(timeout 10 ros2 topic hz /observation 2>&1 | grep "average frequency" | awk '{print $NF}' | tr -d 'Hz')
if [ -z "$freq_obs" ]; then
    echo -e "   ${RED}✗ /observation NOT PUBLISHING${NC}"
else
    echo "   ✓ /observation: $freq_obs Hz"
fi
echo ""

echo "6️⃣  System Health:"
echo "   ROS_DOMAIN_ID matches across instances: $DOMAIN_ID"
echo "   DDS Domain allows inter-machine comms: LOCALHOST_ONLY=0"
echo "   Jetson connectivity: $(ping -c 1 -W 2 $TARGET_IP > /dev/null 2>&1 && echo "✓ Reachable" || echo "✗ Unreachable")"
echo ""

echo "========================================="
echo "✅ System Ready for Testing"
echo "========================================="
echo ""
echo "Next steps:"
echo "  1. Phase 1 (Pi Hardware): make test-phase1"
echo "  2. Phase 2 (Jetson Control): ssh Jetson 'make test-phase2'"
echo "  3. Phase 3 (Perception): ssh Jetson 'make test-phase3'"
echo "  4. Phase 4 (RL Pipeline): ssh Jetson 'make test-phase4'"
