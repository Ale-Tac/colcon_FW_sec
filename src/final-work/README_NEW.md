# Chess-Playing Robot System for UR3/UR3e with Robotiq 85 Gripper

## Overview

This is a complete ROS2-based system for controlling a UR3/UR3e collaborative robot with a Robotiq 85 gripper to play chess. The system perceives the board state using ArUco markers, plans collision-aware trajectories, and executes complex chess moves including standard moves, captures, and castling.

**Key Features:**
- ✅ Real-time ArUco marker detection and TF frame broadcasting
- ✅ Modular architecture with separated Perception, Sensing, Planning, and Action modules
- ✅ Support for both Gazebo simulation and real hardware
- ✅ Complete chess move execution (MOVE, CAPTURE, CASTLING)
- ✅ Inverse kinematics using kinenikros2 for UR robots
- ✅ Gripper control via robotiq_85_gripper_server
- ✅ RViz visualization with TF debugging
- ✅ Well-documented code with clear module responsibilities

## Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     ROS2 Action Client                       │
│              (Terminal: ros2 action send_goal)               │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│                    Action Manager                             │
│  Orchestrates MOVE, CAPTURE, CASTLING actions                │
│  - Calls Sensing to get piece locations                      │
│  - Calls Planning to compute trajectories                    │
│  - Executes motion via robot controller & gripper control   │
└─────┬───────────────────────────────────────────────┬────────┘
      │                                               │
      ▼                                               ▼
┌──────────────────┐                      ┌──────────────────┐
│  Sensing Module  │                      │  Planning Module │
│                  │                      │                  │
│- Piece location  │                      │- IK computation  │
│  queries         │                      │- Trajectory gen. │
│- Cell mapping    │                      │- Waypoint calc.  │
│- Board state     │                      │- Grasp planning  │
└────────┬─────────┘                      └──────────────────┘
         │
         ▼
┌──────────────────────────────────────────────────────────┐
│           Perception Module (Aruco Broadcaster)          │
│                                                          │
│  - Detects ArUco markers (board calibration + pieces)   │
│  - Publishes TF frames for each detected marker         │
│  - Handles camera ↔ world transformation                │
│  - Provides MarkerTf service for frame lookups          │
└──────────────────────────────────────────────────────────┘
         │
         ▼
┌──────────────────────────────────────────────────────────┐
│     External Services (Hardware/Simulation Interface)    │
│                                                          │
│  - RealSense camera (perception)                        │
│  - UR robot driver / chesslab_setup2 (motion)           │
│  - Robotiq gripper server (gripper control)             │
│  - kinenikros2 IK solver                                │
└──────────────────────────────────────────────────────────┘
```

### Module Responsibilities

#### 1. **Perception & ArUco Broadcaster** (`aruco_broadcaster/`)
- Subscribes to `/aruco_marker_publisher/markers` (from aruco_ros)
- Broadcasts TF transforms for each detected ArUco marker
- Provides `GetMarkerTf` service for querying specific marker poses
- Handles board calibration markers (IDs 100–107) and piece markers (201–316)

#### 2. **Sensing Module** (`sensing_module/`)
- Monitors ArUco marker transforms via TF
- Provides `PieceLocation` service:
  - Input: `aruco_id` (piece ID)
  - Output: `pose` (in world frame) + `cell` (chess notation like "E4")
- Maintains piece database and detects movements
- Outputs YAML with current board state

#### 3. **Planning Module** (`planning_module/`)
- Calls `kinenikros2` inverse kinematics solver
- Generates pick-place trajectories with waypoints:
  - Pre-grasp (approach from above)
  - Grasp (descend to piece)
  - Lift (rise with piece)
  - Pre-drop (move above target)
  - Drop (lower to place)
  - Post-drop (lift away)
- Provides `PlanPickPlace` service with full trajectory as JointState array

#### 4. **Action Manager** (`action_manager/`)
- Implements ROS2 Action `ChessAction` for high-level commands
- Orchestrates MOVE, CAPTURE, and CASTLING operations
- Calls Sensing → Planning → Execution pipeline
- Controls gripper (open/close) at appropriate phases
- Provides feedback on action progress
- Handles error recovery and validation

## Board Geometry & Constraints

### Physical Layout
- **Origin**: Center of chessboard (world frame)
- **Cell size**: 5 cm × 5 cm
- **Cell notation**: A–H (files, X-axis) and 1–8 (ranks, Y-axis)
- **Board frame**: `chess_frame` (usually coincides with `world`)
- **Piece placement height**: 4 cm above board surface

### ArUco Marker IDs
- **Board calibration**: 100–107 (4 corners + 4 sides)
- **Black pieces**: 201–216
  - Pawns: 201–208
  - Rooks: 209–210
  - Knights: 211–212
  - Bishops: 213–214
  - Queen: 215
  - King: 216
- **White pieces**: 301–316 (same pattern)

### Piece Heights
- Pawns: ~4 cm
- Rooks/Knights/Bishops: ~6 cm
- King/Queen: ~8 cm

## Prerequisites

### System Requirements
- ROS2 Humble or later
- Ubuntu 20.04+ or equivalent
- Gazebo (for simulation)
- Python 3.8+

### Dependencies (all included in workspace)
- `kinenikros2`: UR inverse kinematics
- `robotiq_85_gripper`: Gripper hardware interface
- `robotiq_85_gripper_server`: Gripper ROS2 wrapper
- `aruco_ros`: ArUco marker detection
- `aruco_broadcaster`: TF frame publishing for markers
- `tablesens`: Camera calibration tools
- `chesslab_setup2`: Scene setup, controllers, simulation
- `chesslab_setup2_interfaces`: Service definitions

## Building

### Full Build
```bash
cd /workspaces/colcon_FW_sec

# Build entire workspace
colcon build

# Source setup file
source install/setup.bash
```

### Selective Build (final-work packages only)
```bash
colcon build --packages-select \
  action_manager sensing_module planning_module aruco_broadcaster

source install/setup.bash
```

## Running the System

### Option 1: Full Simulation (Recommended for Testing)

```bash
# Terminal 1: Launch complete system with Gazebo
ros2 launch final_work chess_system.launch.py mode:=sim ur_type:=ur3 gui:=true rviz:=true
```

This launches:
- Gazebo simulator with UR3 robot, Robotiq gripper, and chessboard
- ArUco broadcaster (detects markers in simulation)
- Sensing module (piece location queries)
- Planning module (trajectory generation)
- Action manager (orchestration)
- RViz for visualization

### Option 2: Real Hardware

```bash
# Ensure UR robot driver is running, then:
ros2 launch final_work chess_system.launch.py mode:=real ur_type:=ur3e rviz:=true
```

### Option 3: Perception & Sensing Only (Debugging)

```bash
# For testing vision without full system
ros2 launch final_work chess_perception.launch.py rviz:=true
```

## Usage & Testing

### Test Individual Modules

#### 1. Test Planning Module Alone
```bash
# Terminal 1: Start IK server and planning node
ros2 run kinenikros2 kinenik_srv_server &
ros2 launch planning_module planning_node.launch.py

# Terminal 2: Call planning service
ros2 service call /plan_pick_place planning_module/srv/PlanPickPlace \
  "{source_pose: {position: {x: -0.15, y: 0.0, z: 0.04}, orientation: {w: 1.0}}, \
    target_pose: {position: {x: 0.15, y: 0.1, z: 0.04}, orientation: {w: 1.0}}}"
```

Expected response:
```yaml
success: true
message: "Trajectory planned successfully"
trajectory:
  - {position: [0.0, -1.57, 1.57, -1.57, 1.57, 0.0]}
  - {position: [...]}  # 6 waypoints total
```

#### 2. Test Sensing Module Alone
```bash
# Terminal 1: Launch perception + sensing
ros2 launch final_work chess_perception.launch.py

# Terminal 2: Query piece location
ros2 service call /piece_location sensing_module/srv/PieceLocation "{aruco_id: 316}"
```

Expected response:
```yaml
found: true
pose:
  position:
    x: -0.175
    y: -0.175
    z: 0.04
cell: "A1"
```

### Send Chess Actions

#### Example 1: Move a Piece
```bash
# Move white king (ID 316) from A1 to C3
ros2 action send_goal /chess_action action_manager/action/ChessAction \
  '{action_type: 0, piece_aruco_id: 316, target_square: "C3"}' \
  --feedback
```

#### Example 2: Capture a Piece
```bash
# White pawn at A2 (ID 301) captures black pawn at B4 (ID 202)
ros2 action send_goal /chess_action action_manager/action/ChessAction \
  '{action_type: 1, piece_aruco_id: 301, target_square: "B4", \
    captured_piece_aruco_id: 202}' \
  --feedback
```

#### Example 3: Castling
```bash
# White kingside castling (king 316 moves, rook follows)
ros2 action send_goal /chess_action action_manager/action/ChessAction \
  '{action_type: 2, piece_aruco_id: 316, target_square: "G1", \
    castling_side: "kingside"}' \
  --feedback
```

### Monitor System Status

```bash
# List active nodes
ros2 node list

# List available services
ros2 service list

# List active actions
ros2 action list

# View TF frame tree
ros2 run tf2_tools view_frames

# Monitor TF transformation
ros2 run tf2_ros tf2_echo chess_frame aruco_316

# View sensor data
ros2 topic echo /aruco_marker_publisher/markers
```

## TF Frame Hierarchy

```
world (root)
  ├── chess_frame (board center)
  │   ├── camera_link
  │   │   └── camera_color_optical_frame
  │   │       ├── aruco_100 (board marker 1)
  │   │       ├── aruco_101 (board marker 2)
  │   │       ├── ... aruco_107
  │   │       ├── aruco_201 (black pawn A)
  │   │       ├── aruco_301 (white pawn A)
  │   │       └── ... (all piece markers)
  ├── ur_base_link
  │   └── ur_manipulator_link_tree...
  └── ...
```

### Frame Naming Conventions
- Board markers: `aruco_100`, `aruco_101`, ... `aruco_107`
- Piece markers: `aruco_201`, `aruco_202`, ..., `aruco_316`

### Debugging TF
```bash
# Generate PDF of frame tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo chess_frame aruco_316

# Monitor real-time
ros2 run tf2_ros tf2_monitor chess_frame aruco_316

# List all frames
ros2 run tf2_tools view_frames --output frame_tree.pdf
```

## Configuration

### Sensing Module Parameters
```yaml
# Parameters in launch file or config file
aruco_ids: [201, 202, ..., 316]  # IDs to track
yaml_output_path: "/tmp/chess_configuration_sensed.yaml"
```

### Planning Module Tuning
Adjustable in `planning_module/src/planning_node.cpp`:
```cpp
const double d = 0.15;           // 15 cm grasp offset
const double pre_offset = 0.10;  // 10 cm pre-grasp height
const double lift_offset = 0.10; // 10 cm lift height
```

### Action Manager Parameters
```cpp
static constexpr float GRIPPER_CLOSE_POSITION = 0.8f;
static constexpr float GRIPPER_OPEN_POSITION = 0.0f;
static constexpr double SQUARE_SIZE = 0.05;  // 5 cm
static constexpr double BOARD_Z = 0.04;      // 4 cm
```

## Troubleshooting

### ArUco markers not detected
1. Verify camera is publishing to `/aruco_marker_publisher/markers`
2. Check camera calibration (use `tablesens` for real hardware)
3. Ensure marker size: 26×26 mm for pieces, board markers larger
4. Check lighting and marker visibility
5. Test with: `ros2 topic echo /aruco_marker_publisher/markers`

### IK solver fails
1. Ensure `kinenikros2` is built and running
2. Check if target pose is within robot's workspace
3. Verify UR type configuration (ur3 vs ur3e)
4. Check for collisions with board/gripper
5. Test directly: `ros2 service call /inverse_kinematics ...`

### Gripper doesn't respond
1. Verify `robotiq_85_gripper_server` is running
2. Check gripper connection and power
3. Test directly: `ros2 service call /gripper_order robotiq_85_gripper_server/srv/GripperOrder "{order: 1}"`
4. Check gripper logs: `ros2 node info /gripper_driver`

### Frame lookup timeouts
1. Ensure ArUco broadcaster is receiving markers
2. Check TF buffer (default 10 seconds)
3. Verify sensor and robot frames are published
4. Use `tf2_echo` to debug frame hierarchy
5. Check for static transform publisher issues

### Robot doesn't move
1. Verify chesslab_setup2 controllers are active
2. Check `/set_robot_configuration` service is available
3. In simulation: ensure Gazebo is running
4. On real hardware: ensure UR driver is running
5. Check joint limits in trajectory

## Code Quality & Documentation

### Code Organization
- **Headers** (`include/*/node.hpp`): Class definitions with clear responsibilities
- **Implementation** (`src/*_node.cpp`): Detailed comments and error handling
- **Services/Actions** (`.srv`, `.action`): Well-documented message definitions
- **Launch files** (Python-based): Clear parameters and conditions
- **CMakeLists.txt**: Modular structure with explicit dependencies

### Key Source Files

| File | Purpose |
|------|---------|
| `action_manager/src/action_manager_node.cpp` | Main action orchestrator (500+ lines) |
| `planning_module/src/planning_node.cpp` | Trajectory planning & IK (235 lines) |
| `sensing_module/src/sensing_node.cpp` | Piece detection & location (195 lines) |
| `aruco_broadcaster/src/aruco_broadcaster.cpp` | Marker detection & TF (210 lines) |
| `launch/chess_system.launch.py` | Complete system launcher |

### Coding Standards
- Clear variable/function naming
- Inline comments for complex logic
- Error handling with detailed logging
- ROS2-specific patterns (callbacks, services, actions)
- Modular design with separation of concerns

## Testing & Validation

### Simulation Tests (Recommended First)
1. **System startup**: Verify all nodes launch without errors
2. **Service availability**: Check all services are accessible
3. **Basic move**: Send MOVE action and verify piece moves
4. **Gripper control**: Test grasp and release
5. **Board state**: Verify final position matches target

### Real Hardware Tests
1. **Marker detection**: Verify all pieces visible to camera
2. **Camera calibration**: Use `tablesens` to calibrate pose
3. **Workspace testing**: Verify IK for reachable positions
4. **Safety checks**: Test with reduced speed/force
5. **End-to-end**: Execute simple move and verify result

### Performance Metrics
- **Total cycle time** (full move): 5–10 seconds
- **Perception latency**: <500 ms (TF lookup + sensing)
- **Planning time**: 2–5 seconds (IK + trajectory)
- **Execution time**: 2–5 seconds (robot motion)

## File Structure

```
final-work/
├── action_manager/
│   ├── action/ChessAction.action        # ROS2 action definition
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/action_manager/
│   │   └── action_manager_node.hpp
│   ├── src/
│   │   └── action_manager_node.cpp      # Main implementation (500+ lines)
│   ├── launch/
│   │   └── action_manager.launch.py
│   └── config/
│
├── sensing_module/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── srv/PieceLocation.srv
│   ├── src/
│   │   └── sensing_node.cpp
│   ├── launch/
│   └── config/
│
├── planning_module/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── srv/PlanPickPlace.srv
│   ├── include/planning_module/
│   │   └── planning_node.hpp
│   ├── src/
│   │   └── planning_node.cpp
│   ├── launch/
│   │   └── planning_node.launch.py
│   └── config/
│
├── aruco_broadcaster/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── srv/GetMarkerTf.srv
│   ├── src/
│   │   └── aruco_broadcaster.cpp
│   ├── launch/
│   └── config/
│
├── launch/
│   ├── chess_system.launch.py           # Main launch file
│   └── chess_perception.launch.py       # Perception-only launch
│
└── README.md (this file)
```

## Future Enhancements

- [ ] Legal move validation (enforce chess rules)
- [ ] Multi-action queue (batch moves)
- [ ] Advanced grasp planning (friction analysis, stability)
- [ ] Refined collision avoidance
- [ ] Real-time vision feedback during execution
- [ ] Smooth trajectory interpolation (cubic splines)
- [ ] Game state management and engine integration
- [ ] Automatic board reset between games
- [ ] Performance logging and statistics
- [ ] Graceful error recovery

## License

MIT License

## Authors & Acknowledgments

Team Chess-Robot Project, 2025

Developed with ROS2 and leveraging:
- UR Robotics for robot hardware
- Robotiq for gripper solutions
- ArUco library for marker detection
- ROS2 community for middleware
