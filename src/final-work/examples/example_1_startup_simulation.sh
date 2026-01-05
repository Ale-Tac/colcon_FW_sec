#!/bin/bash

# Example 1: Full system simulation startup
# This script demonstrates the complete startup sequence for the chess robot

echo "================================================"
echo "Chess Robot - Full Simulation Startup Example"
echo "================================================"
echo ""
echo "This script shows how to launch the complete system in simulation mode."
echo ""
echo "Prerequisites:"
echo "  1. ROS2 Humble installed"
echo "  2. Gazebo installed"
echo "  3. All packages built with: colcon build"
echo ""
echo "Usage:"
echo "  bash examples/example_1_startup_simulation.sh"
echo ""
echo "What it does:"
echo "  - Starts Gazebo simulator with robot, gripper, and chess board"
echo "  - Launches perception (ArUco detection)"
echo "  - Launches sensing module (piece location tracking)"
echo "  - Launches planning module (trajectory generation)"
echo "  - Launches action manager (chess move orchestration)"
echo "  - Opens RViz for visualization"
echo ""
echo "================================================"
echo ""

# Step 1: Source ROS2
echo "[1/5] Sourcing ROS2 environment..."
# source /opt/ros/humble/setup.bash
# source install/setup.bash

# Step 2: Launch the system
echo "[2/5] Launching full chess system in simulation..."
echo ""
echo "Command to execute:"
echo ""
echo "  ros2 launch final_work chess_system.launch.py \\"
echo "    mode:=sim \\"
echo "    ur_type:=ur3 \\"
echo "    rviz:=true \\"
echo "    gui:=true"
echo ""
echo "Wait for all nodes to start (you should see messages from each module)"
echo ""

# Step 3: Information for user
echo "[3/5] System will provide:"
echo "  - Gazebo: Robot simulation with physics"
echo "  - RViz: TF tree visualization and marker displays"
echo "  - Action manager: Ready to accept chess commands"
echo ""

echo "[4/5] Once running, in another terminal, try a test command:"
echo ""
echo "  ros2 action send_goal /chess_action action_manager/action/ChessAction \\"
echo "    \"{action_type: 0, piece_aruco_id: 316, target_square: 'E2', \" \\"
echo "    \"captured_piece_aruco_id: 0, castling_side: ''}\" \\"
echo "    --feedback"
echo ""

echo "[5/5] To stop the system:"
echo "  Press Ctrl+C in the terminal"
echo ""
echo "================================================"
echo "For more examples, see other .sh files in this directory"
echo "For detailed documentation, see ../README.md"
echo "================================================"
