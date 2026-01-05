# Chess-Playing Robot System with UR3/UR3e and Robotiq 85 Gripper

A complete ROS2 system for autonomous chess piece manipulation on a physical or simulated chessboard using a UR industrial robot arm with an adaptive gripper.

## Project Overview

This system enables a UR3/UR3e collaborative robot equipped with a Robotiq 85 gripper to:
- **MOVE**: Relocate a chess piece from its current position to an empty target square
- **CAPTURE**: Remove an opponent piece and move the attacking piece to the target square
- **CASTLING**: Perform coordinated king and rook movements (advanced feature)

The system operates in two modes:
- **Simulation (Gazebo)**: Full physics simulation with visual feedback
- **Real Hardware**: Actual robot control with RealSense camera for vision

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Action Manager (orchestrator)             │
│              - Receives MOVE/CAPTURE/CASTLING commands      │
│              - Coordinates all subsystems                    │
└──────────────┬──────────────┬──────────────┬────────────────┘
               │              │              │
       ┌───────▼──┐    ┌──────▼────┐  ┌─────▼──────┐
       │ Sensing  │    │ Planning  │  │  Execution │
       │ Module   │    │ Module    │  │  (Gripper) │
       └───────┬──┘    └──────┬────┘  └─────▲──────┘
               │              │              │
       ┌───────▼──────────────▼──────────────▼──────┐
       │   Perception (TF, ArUco Markers, Frames)   │
       └──────────────────────┬─────────────────────┘
                              │
       ┌──────────────────────▼──────────────────────┐
       │  Robot/Scene Control (chesslab_setup2)      │
       │  - Gazebo Simulation                        │
       │  - Physical Robot Control                   │
       └──────────────────────────────────────────────┘
```

## Directory Structure

```
src/
├── final-work/                          # EDITABLE: Main application code
│   ├── action_manager/                  # High-level action orchestration
│   │   ├── action/ChessAction.action    # ROS2 action definition
│   │   ├── src/action_manager_node.cpp  # Main orchestrator
│   │   ├── include/action_manager/      # Header files
│   │   ├── launch/                      # Launch files
│   │   └── CMakeLists.txt
│   │
│   ├── sensing_module/                  # Perception & piece location
│   │   ├── src/sensing_node.cpp         # Piece detection from TF frames
│   │   ├── srv/PieceLocation.srv        # Query piece positions
│   │   └── CMakeLists.txt
│   │
│   ├── planning_module/                 # Trajectory planning
│   │   ├── src/planning_node.cpp        # Waypoint & IK planning
│   │   ├── srv/PlanPickPlace.srv        # Plan pick-place motion
│   │   └── CMakeLists.txt
│   │
│   ├── aruco_broadcaster/               # TF publishing for markers
│   │   ├── src/aruco_broadcaster.cpp    # Broadcasts ArUco → TF
│   │   ├── srv/GetMarkerTf.srv          # Query marker transforms
│   │   └── CMakeLists.txt
│   │
│   └── launch/                          # Master launch files
│       ├── chess_system.launch.py       # Full system (sim or real)
│       └── chess_perception.launch.py   # Perception only (debugging)
│
├── chesslab_setup2/                     # EXTERNAL: Robot/scene setup
│   ├── launch/chesslab_setup2_demo.launch.py
│   ├── urdf/                            # Robot/board URDF
│   └── worlds/                          # Gazebo world files
│
├── chesslab_setup2_interfaces/          # EXTERNAL: Service definitions
│   ├── srv/SetRobConf.srv               # Set robot joint configuration
│   ├── srv/SetObjPose.srv               # Set object pose in sim
│   └── srv/InverseKinematicsSimple.srv  # Basic IK
│
├── kinenikros2/                         # EXTERNAL: UR kinematics
├── robotiq_85_gripper/                  # EXTERNAL: Gripper hardware
├── robotiq_85_gripper_server/           # EXTERNAL: Gripper control
├── aruco_ros/                           # EXTERNAL: ArUco detection
└── tablesens/                           # EXTERNAL: Camera calibration
```

## Hardware Setup

### Physical Configuration
- **Chessboard**: 8×8 squares, each 5 cm × 5 cm
- **Board Markers**: ArUco IDs 100–107 (26×26 mm, for camera calibration)
- **World Frame**: Located at chessboard center
- **Cell Notation**: A1 (bottom-left) to H8 (top-right)

### Piece Markers
Each piece has an ArUco marker glued to its top surface:

| Piece Type      | Black IDs    | White IDs    | Height |
|-----------------|--------------|--------------|--------|
| Pawns           | 201–208      | 301–308      | 4 cm   |
| Rooks           | 209–210      | 309–310      | 6 cm   |
| Knights         | 211–212      | 311–312      | 6 cm   |
| Bishops         | 213–214      | 313–314      | 6 cm   |
| Queen           | 215          | 315          | 8 cm   |
| King            | 216          | 316          | 8 cm   |

### Robot & Gripper
- **Robot**: UR3 or UR3e (6-DOF collaborative arm)
- **Gripper**: Robotiq 85 (adaptive parallel gripper)
- **Camera**: RealSense D455 (for vision and calibration)
- **Base Pose**: Obtained via calibration using board markers

## Building

### Prerequisites
```bash
# Ubuntu 22.04 LTS with ROS2 Humble
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros \
    ros-humble-tf2-ros \
    ros-humble-rclcpp \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    build-essential git
```

### Build All Packages
```bash
cd /workspaces/colcon_FW_sec

# Build everything
colcon build

# Or build only final-work modules
colcon build --packages-select \
    aruco_broadcaster \
    sensing_module \
    planning_module \
    action_manager
```

### Build Output
```
install/
├── action_manager/
├── aruco_broadcaster/
├── planning_module/
├── sensing_module/
└── [other packages]
```

## Running the System

### Setup Environment
```bash
cd /workspaces/colcon_FW_sec
source install/setup.bash
```

### Mode 1: Full Simulation (Gazebo)
Launch complete system with simulation:
```bash
ros2 launch final_work chess_system.launch.py mode:=sim ur_type:=ur3 gui:=true
```

This starts:
- Gazebo simulator with robot, gripper, board, and pieces
- ArUco marker detection (simulated)
- Sensing module
- Planning module
- Action manager
- RViz visualization

### Mode 2: Real Hardware
```bash
ros2 launch final_work chess_system.launch.py mode:=real ur_type:=ur3e
```

This starts:
- Real robot driver (UR robot)
- RealSense camera node
- Camera calibration via tablesens
- All perception, sensing, planning, and control modules

### Mode 3: Perception Only (Debugging)
Test perception and sensing without robot motion:
```bash
ros2 launch final_work chess_perception.launch.py rviz:=true
```

Shows:
- ArUco marker detection
- Piece location tracking
- TF frame visualization

## Using the System

### Example 1: Move a Piece

**Scenario**: Move white king (ArUco 316) from E1 to E2

```bash
# Terminal: Send chess action
ros2 action send_goal /chess_action action_manager/action/ChessAction \
  "{
    action_type: 0,
    piece_aruco_id: 316,
    target_square: 'E2',
    captured_piece_aruco_id: 0,
    castling_side: ''
  }" \
  --feedback
```

**Expected Output**:
```
[action_manager] Executing chess action
[action_manager] [MOVE] Step 1/5: Sensing current position
[action_manager] Piece 316 at E1
[action_manager] [MOVE] Step 2/5: Computing target pose
[action_manager] [MOVE] Step 3/5: Planning trajectory
[action_manager] Trajectory planned with 6 waypoints
[action_manager] [MOVE] Step 4/5: Executing motion
[action_manager] Gripper closed
[action_manager] Gripper opened
[action_manager] [MOVE] Step 5/5: Verifying result
[action_manager] Final position: E2
[action_manager] SUCCESS: Piece at target square
```

### Example 2: Capture a Piece

**Scenario**: White pawn at E4 captures black pawn at D5

```bash
ros2 action send_goal /chess_action action_manager/action/ChessAction \
  "{
    action_type: 1,
    piece_aruco_id: 305,
    target_square: 'D5',
    captured_piece_aruco_id: 204,
    castling_side: ''
  }" \
  --feedback
```

### Example 3: Query Piece Location (Service)

```bash
ros2 service call /piece_location sensing_module/srv/PieceLocation \
  "{aruco_id: 316}"
```

**Response**:
```
found: true
pose:
  position:
    x: 0.0
    y: -0.175
    z: 0.04
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
cell: 'E1'
```

### Example 4: Test Planning

```bash
ros2 service call /plan_pick_place planning_module/srv/PlanPickPlace \
  "{
    source_pose: {position: {x: 0.0, y: -0.175, z: 0.04}, orientation: {w: 1.0}},
    target_pose: {position: {x: 0.0, y: -0.125, z: 0.04}, orientation: {w: 1.0}}
  }"
```

## Module Details

### 1. ArUco Broadcaster
**Purpose**: Detect ArUco markers and broadcast TF frames

**Key Files**:
- `aruco_broadcaster/src/aruco_broadcaster.cpp`: Subscribes to `/aruco_marker_publisher/markers`, broadcasts TF

**Services**:
- `get_marker_Tf`: Request transform between two frames for a specific marker

**Key Parameters** (config/aruco_broadcaster.yaml):
```yaml
aruco_broadcaster:
  ros__parameters:
    markerList: [100, 101, 102, 103, 104, 105, 106, 107, 201, 202, ...]
    camera_frame: "camera_color_optical_frame"
    aruco_frame: "aruco"
```

### 2. Sensing Module
**Purpose**: Convert TF frames to piece locations and cell positions

**Key Files**:
- `sensing_module/src/sensing_node.cpp`: Listens to TF transforms, maps poses to chess cells

**Services**:
- `piece_location`: Input `aruco_id` → Output `pose` + `cell` (e.g., "E1")

**Algorithm**:
1. Listen for TF frame: `world` → `aruco_<ID>`
2. Extract XY position
3. Map to nearest chess cell: $\text{cell} = (x/0.05 + 3.5, y/0.05 + 3.5)$
4. Output pose and cell name

**Configuration** (launch file):
```python
parameters=[{
    "aruco_ids": [200-316],  # All piece markers
    "yaml_output_path": "/tmp/chess_configuration_sensed.yaml"
}]
```

### 3. Planning Module
**Purpose**: Generate robot trajectories using IK solver

**Key Files**:
- `planning_module/src/planning_node.cpp`: Computes pick-place waypoints and IK solutions

**Services**:
- `plan_pick_place`: Input source & target poses → Output trajectory (JointState array)

**Trajectory Generation**:
1. **Pre-grasp**: Move above source (z + 0.10 m)
2. **Grasp**: Move to source position (gripper orientation: pointing down)
3. **Lift**: Raise z by 0.10 m
4. **Pre-drop**: Move above target (z + 0.10 m)
5. **Drop**: Move to target position
6. **Post-drop**: Lift slightly (z + 0.05 m)

**IK Call**:
- Calls `/inverse_kinematics` (kinenikros2 service) for each waypoint
- Returns first valid solution

### 4. Action Manager
**Purpose**: Orchestrate high-level chess moves

**Key Files**:
- `action_manager/src/action_manager_node.cpp`: Main orchestrator
- `action_manager/action/ChessAction.action`: Action definition

**Action Interface**:
```
Input:
  - action_type: 0=MOVE, 1=CAPTURE, 2=CASTLING
  - piece_aruco_id: ArUco ID to move
  - target_square: Cell notation (e.g., "E4")
  - captured_piece_aruco_id: For CAPTURE mode
  - castling_side: "kingside" or "queenside" for CASTLING

Feedback:
  - progress: 0.0-100.0
  - status: Current stage description

Result:
  - success: true/false
  - message: Result description
  - final_square: Where piece ended up
```

**Execution Flow**:
1. **Sensing** (10%): Get current piece pose via `/piece_location`
2. **Planning** (30%): Convert target cell to pose, call `/plan_pick_place`
3. **Execution** (60%): Execute trajectory, control gripper
4. **Verification** (90%): Re-sense position, confirm at target
5. **Complete** (100%): Return result

## Launch Files Reference

### chess_system.launch.py (Main)
Full system launch with arguments:

| Argument | Default | Options | Description |
|----------|---------|---------|-------------|
| `mode` | `sim` | `sim`, `real` | Simulation or real hardware |
| `ur_type` | `ur3` | `ur3`, `ur3e` | UR robot model |
| `rviz` | `true` | `true`, `false` | Launch RViz visualization |
| `gui` | `true` | `true`, `false` | Gazebo GUI (sim only) |

**Example**:
```bash
ros2 launch final_work chess_system.launch.py mode:=sim ur_type:=ur3e gui:=true rviz:=true
```

### chess_perception.launch.py (Debugging)
Perception and sensing only:
```bash
ros2 launch final_work chess_perception.launch.py rviz:=true
```

## Troubleshooting

### 1. Markers Not Detected
```bash
# Check if aruco_marker_publisher is running
ros2 topic list | grep aruco

# Check TF frames
ros2 run tf2_tools view_frames
cd /tmp && evince frames.pdf  # View TF tree
```

### 2. Piece Location Service Returns "not-found"
```bash
# Verify TF transform exists
ros2 run tf2_ros tf_echo world aruco_316

# Check sensing_node logs
ros2 launch final_work chess_perception.launch.py
# Look for "[piece_location] id 316 -> not-found"
```

### 3. Planning Fails (IK No Solution)
- Robot workspace may not reach target pose
- Check if pose is within UR3/UR3e reach (radius ~1.3 m for UR3)
- Verify target pose is above the board (z > 0.02)

### 4. Gripper Not Responding
```bash
# Check gripper service
ros2 service list | grep gripper
ros2 service call /gripper_order robotiq_85_gripper_server/srv/GripperOrder "{order: 1}"
```

### 5. Robot Collisions
- Ensure board is clear of obstacles
- Check chesslab_setup2 MoveIt configuration
- Use RViz to visualize collisions

## Code Quality & Best Practices

### Naming Conventions
- **Services**: `snake_case` (e.g., `piece_location`, `plan_pick_place`)
- **Actions**: `PascalCase` (e.g., `ChessAction`)
- **Topics**: `snake_case` (e.g., `/aruco_marker_publisher/markers`)
- **Frames**: `snake_case` (e.g., `world`, `chess_frame`, `aruco_316`)

### Logging
All nodes use ROS2 logging with appropriate levels:
```cpp
RCLCPP_DEBUG(get_logger(), "Detailed debug info");
RCLCPP_INFO(get_logger(), "General information");
RCLCPP_WARN(get_logger(), "Warning condition");
RCLCPP_ERROR(get_logger(), "Error occurred");
```

### Configuration
Parameters are declared in launch files:
```python
parameters=[{
    "param_name": default_value,
    "aruco_ids": [200, 201, ...],
}]
```

## Performance Notes

- **Sensing Update Rate**: 2 Hz (500 ms cycle)
- **Planning Time**: 2-5 seconds (depends on IK solver)
- **Execution Time**: 5-10 seconds (pick-place with gripper control)
- **Total Move Time**: 10-20 seconds per chess move
- **Accuracy**: ±5 mm (depends on marker detection and IK solver)

## Testing Checklist

### Simulation Tests
- [ ] Full system launches without errors
- [ ] Can move a piece from A1 to B1
- [ ] Can move a piece from A1 to H8
- [ ] Capture move removes captured piece
- [ ] Perception correctly identifies all 32 piece positions
- [ ] RViz shows correct TF frames and piece poses

### Real Hardware Tests
- [ ] Robot calibration successful
- [ ] Camera calibration successful
- [ ] All piece markers detected
- [ ] Gripper opens/closes smoothly
- [ ] Can move a piece successfully
- [ ] Piece ends up at target cell

## Future Improvements

1. **Chess Rules Validation**: Verify moves are legal according to chess rules
2. **Advanced Planning**: Collision-aware trajectory planning with MoveIt2
3. **Dynamic Obstacle Avoidance**: Detect and avoid obstacles in workspace
4. **Multiple Gripper Types**: Support different end-effectors
5. **AI Integration**: Connect to chess engines (Stockfish, etc.)
6. **Real-time Performance**: Optimize for faster move execution
7. **Force Control**: Use force feedback for robust grasping
8. **Piece Feedback**: Detect when pieces are moved by opponent (game detection)

## References

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [UR Robot Documentation](https://www.universal-robots.com/en/)
- [Robotiq Gripper Documentation](https://robotiq.com/)
- [ArUco OpenCV Documentation](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html)

## License

MIT License - See LICENSE file in final-work/

## Contact & Support

For issues or questions:
1. Check the Troubleshooting section above
2. Review node logs: `ros2 node list` and `ros2 node info <node>`
3. Check service availability: `ros2 service list`
4. Inspect TF tree: `ros2 run tf2_tools view_frames`
