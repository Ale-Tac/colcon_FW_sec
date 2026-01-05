# Build and Run Guide - Chess Robot System

**Complete instructions for building and running the chess-playing robot system**

---

## 📋 Prerequisites

### System Requirements
- Ubuntu 20.04+ (tested on 24.04)
- ROS2 Humble installed
- Python 3.8+
- colcon build tool

### Required Packages
Ensure these are symlinked into the src/ directory:
```bash
# From data folder to workspace
ln -s ~/data/all_introduction_to_ros2_packages/kinenikros2 src/
ln -s ~/data/all_introduction_to_ros2_packages/robotiq_85_gripper src/
ln -s ~/data/all_introduction_to_ros2_packages/robotiq_85_gripper_server src/
ln -s ~/data/all_introduction_to_ros2_packages/aruco_ros src/
ln -s ~/data/all_introduction_to_ros2_packages/tablesens src/
ln -s ~/data/all_introduction_to_ros2_packages/chesslab_setup2 src/
ln -s ~/data/all_introduction_to_ros2_packages/chesslab_setup2_interfaces src/
```

---

## 🔨 Building the System

### Step 1: Navigate to Workspace
```bash
cd /workspaces/colcon_FW_sec
```

### Step 2: Source ROS2
```bash
source /opt/ros/humble/setup.bash
```

### Step 3: Full Build
```bash
colcon build
```

**Expected output**: All packages build without errors

### Step 4: Source Installation
```bash
source install/setup.bash
```

### Troubleshooting Build Issues

**Issue**: Package not found
```bash
# Check if all dependencies are present
ls src/
# Should show: final-work, kinenikros2, robotiq_85_gripper, etc.
```

**Issue**: Build fails for sensing_module
```bash
# Rebuild sensing module only
colcon build --packages-select sensing_module

# Check for C++ syntax errors
cat src/final-work/sensing_module/src/sensing_node.cpp | head -100
```

---

## 🚀 Running the System

### Mode 1: Full Simulation (Recommended to Start)

**Terminal 1 - Main System**:
```bash
cd /workspaces/colcon_FW_sec
source install/setup.bash
ros2 launch final_work chess_system.launch.py mode:=sim ur_type:=ur3e rviz:=true
```

**Expected output**:
- Gazebo window opens with robot and chessboard
- RViz window shows robot, pieces, and coordinate frames
- Console shows "ActionManager initialized successfully"

**Wait ~10 seconds for all nodes to start**

**Terminal 2 - Send Chess Move**:
```bash
source install/setup.bash

# Move white pawn from A2 to A3
ros2 action send_goal /chess_action action_manager/action/ChessAction \
  "{action_type: 0, piece_aruco_id: 301, target_square: 'A3'}" --feedback
```

**Expected result**:
- Action feedback shows progress updates
- Robot gripper should close and move (in simulation)
- Gripper should open at target square
- Action completes with success message

---

### Mode 2: Perception Only (For Debugging)

Use this to test sensing without full robot simulation:

**Terminal 1**:
```bash
source install/setup.bash
ros2 launch final_work chess_perception.launch.py
```

This starts:
- ArUco marker detection
- Sensing module (piece localization)
- RViz visualization

**Terminal 2 - Query Pieces**:
```bash
source install/setup.bash

# Query piece location
ros2 service call /piece_location sensing_module/srv/PieceLocation "{aruco_id: 316}"
```

**Expected output** (after bug is fixed):
```
found: true
pose:
  position:
    x: 0.025
    y: -0.175
    z: 0.04
  orientation: ...
cell: 'E1'
```

---

### Mode 3: Real Hardware (When Available)

**Prerequisites**:
- UR robot powered on and ready
- RealSense camera connected and calibrated
- ur_robot_driver running

**Terminal 1**:
```bash
source install/setup.bash
ros2 launch final_work chess_system.launch.py mode:=real ur_type:=ur3e
```

This starts:
- ArUco detection from RealSense camera
- Sensing module
- Planning module
- Action manager
- No Gazebo or simulation

**Terminal 2 - Send Move**:
```bash
source install/setup.bash
ros2 action send_goal /chess_action action_manager/action/ChessAction \
  "{action_type: 0, piece_aruco_id: 301, target_square: 'A3'}" --feedback
```

---

## ✅ Testing Checklist

### Basic Sanity Tests (Run in order)

#### Test 1: Build Success
```bash
colcon build
# Should complete without errors
```

#### Test 2: Node Launch
```bash
# Terminal 1
ros2 launch final_work chess_perception.launch.py

# Terminal 2
ros2 topic list | grep aruco
# Should show /aruco_marker_publisher/markers
```

#### Test 3: Piece Sensing
```bash
# With chess_perception.launch.py running:
ros2 service call /piece_location sensing_module/srv/PieceLocation "{aruco_id: 316}"
```

**Check**: Does it return any cell? (Even if wrong, at least detection works)

#### Test 4: Coordinate Bug Check
```bash
# If Test 3 returned "A4" for piece 316, you need to fix coordinates
# See: COORDINATE_FIX_GUIDE.md and QUICK_FIXES.md
```

#### Test 5: Full System Launch
```bash
ros2 launch final_work chess_system.launch.py mode:=sim
# Wait 10 seconds
# Should see Gazebo and robot loaded
```

#### Test 6: Action Server
```bash
# Terminal 1
ros2 launch final_work chess_system.launch.py mode:=sim
# Wait 10 seconds

# Terminal 2
ros2 action list | grep chess
# Should show /chess_action

ros2 action send_goal /chess_action action_manager/action/ChessAction \
  "{action_type: 0, piece_aruco_id: 301, target_square: 'A3'}" --feedback
```

**Expected**: Action executes and returns success/failure

---

## 🔧 Practical Examples

### Example 1: Move White Pawn
```bash
ros2 action send_goal /chess_action action_manager/action/ChessAction \
  "{action_type: 0, piece_aruco_id: 301, target_square: 'A3'}" --feedback
```

### Example 2: Move White King
```bash
ros2 action send_goal /chess_action action_manager/action/ChessAction \
  "{action_type: 0, piece_aruco_id: 316, target_square: 'E2'}" --feedback
```

### Example 3: Capture Black Piece
```bash
ros2 action send_goal /chess_action action_manager/action/ChessAction \
  "{action_type: 1, piece_aruco_id: 301, target_square: 'A4', captured_piece_aruco_id: 201}" --feedback
```

### Example 4: Castling
```bash
ros2 action send_goal /chess_action action_manager/action/ChessAction \
  "{action_type: 2, piece_aruco_id: 316, target_square: 'G1', castling_side: 'kingside'}" --feedback
```

---

## 🐛 Fixing the Coordinate Bug

**If pieces report wrong cells**, follow these steps:

### Step 1: Understand the Bug
- Piece 316 (white king) reports as "A4" instead of "E1"
- Root cause: `pose_to_cell()` formula in sensing_node.cpp

### Step 2: Identify Pattern
```bash
bash run_sensing_diagnostic.sh  # See SENSING_COORDINATE_ALTERNATIVES.hpp for this
```

Note the actual x, y values returned. Compare with expected.

### Step 3: Apply Fix
Choose the correct formula from QUICK_FIXES.md:
- Fix 1: Axes swapped (most common)
- Fix 2: X axis inverted
- Fix 3: Y axis inverted
- Fix 4: Both swapped and inverted
- Fix 5: Board origin at corner

### Step 4: Rebuild
```bash
colcon build --packages-select sensing_module
source install/setup.bash
```

### Step 5: Verify
```bash
ros2 run sensing_module sensing_node
ros2 service call /piece_location sensing_module/srv/PieceLocation "{aruco_id: 316}"
```

Should now report "E1" (or whatever the correct cell is)

---

## 📊 Expected Output Examples

### Successful Piece Sensing
```
ros2 service call /piece_location sensing_module/srv/PieceLocation "{aruco_id: 316}"
---
found: true
pose:
  position:
    x: 0.025
    y: -0.175
    z: 0.04
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
cell: 'E1'
```

### Successful Action Execution
```
ros2 action send_goal /chess_action action_manager/action/ChessAction \
  "{action_type: 0, piece_aruco_id: 301, target_square: 'A3'}" --feedback

Sending goal:
     action_type: 0
     piece_aruco_id: 301
     target_square: A3
     captured_piece_aruco_id: 0
     castling_side: ''

Goal accepted with ID: ...

Feedback:
  progress: 10.0
  status: Sensing current position

Feedback:
  progress: 20.0
  status: Computing target pose

[... more feedback ...]

Feedback:
  progress: 100.0
  status: Move complete

Result:
  success: true
  message: Chess action completed successfully
  final_square: A3
```

---

## 🚨 Common Issues

### Issue 1: "piece_location service not available"
**Cause**: Sensing node not running  
**Fix**:
```bash
ros2 run sensing_module sensing_node &
# Then try service call
```

### Issue 2: "piece_location returns 'not-found'"
**Cause**: Piece not detected by ArUco  
**Fix**:
```bash
# Check if markers are being detected
ros2 topic echo /aruco_marker_publisher/markers
# Should show marker_array with IDs 201-216
```

### Issue 3: "Wrong cell reported (e.g., A4 instead of E1)"
**Cause**: Coordinate transformation bug  
**Fix**: Follow "Fixing the Coordinate Bug" section above

### Issue 4: "Action never completes"
**Cause**: IK solver not available or planning failed  
**Fix**:
```bash
# Ensure kinenikros2 is running
ros2 run kinenikros2 kinenik_srv_server &
# Then retry action
```

### Issue 5: "Build fails for action_manager"
**Cause**: Action interface not generated  
**Fix**:
```bash
rm -rf build/ install/ log/
colcon build --packages-select action_manager
```

---

## 📈 Performance Notes

- **Build time**: 5-10 minutes (first time), 1-2 minutes (incremental)
- **System startup**: ~10 seconds (all nodes starting)
- **Action execution**: ~5-15 seconds (depending on trajectory)
- **Sensing update rate**: 0.5 seconds (500ms timer)

---

## 🔗 Related Documentation

- `COORDINATE_FIX_GUIDE.md` - Detailed coordinate bug analysis
- `QUICK_FIXES.md` - Copy-paste solutions
- `COMPLETE_SYSTEM_SUMMARY.md` - Full system overview
- `SENSING_COORDINATE_DEBUG.md` - Technical deep dive

---

**Last Updated**: January 3, 2026  
**System Status**: 🟡 Core system ready, coordinate issue pending fix
