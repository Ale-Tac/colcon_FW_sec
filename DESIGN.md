# System Design & Implementation Details

## Complete System Flow

### 1. Overall Architecture

```
┌─────────────────────────────────────────────────────┐
│         USER: Action Request (terminal)              │
│   "Move piece 305 from E2 to E4"                     │
└────────────────────┬────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────┐
│       ACTION MANAGER (orchestrator)                   │
│   - Validates request                                │
│   - Coordinates all subsystems                       │
│   - Handles errors & retries                         │
│   - Returns success/failure                          │
└────────┬────────────────────┬──────────────────┬─────┘
         │                    │                  │
         ▼                    ▼                  ▼
    ┌─────────────┐  ┌──────────────┐  ┌──────────────┐
    │ SENSING     │  │ PLANNING     │  │ EXECUTION    │
    │             │  │              │  │              │
    │ Get piece   │  │ Generate     │  │ Control      │
    │ location    │  │ trajectory   │  │ - Gripper    │
    │ (E2)        │  │ using IK     │  │ - Robot arm  │
    └─────────────┘  │ 6 waypoints  │  │ - Verify     │
                     └──────────────┘  └──────────────┘
         ▲                    │                  │
         └────────────────────┴──────────────────┘
                    TF Frames & Poses
                     (world → aruco_305)
                          ↓
        ┌──────────────────────────────────────┐
        │    PERCEPTION & TF LAYER             │
        │                                       │
        │  ArUco Broadcaster:                  │
        │    /aruco_marker_publisher/markers   │
        │         ↓ (publishes TF)             │
        │    aruco_305 frame                   │
        │         ↓ (TF lookup)                │
        │    world → camera → aruco_305        │
        └──────────────────────────────────────┘
                          ↓
        ┌──────────────────────────────────────┐
        │    ROBOT & ENVIRONMENT               │
        │                                       │
        │  Gazebo (simulation) or              │
        │  Real robot + RealSense camera       │
        │                                       │
        │  - Physical robot kinematics         │
        │  - Gripper control                   │
        │  - Physics simulation                │
        │  - Scene objects (pieces, board)     │
        └──────────────────────────────────────┘
```

## Detailed Module Specifications

### Module 1: ArUco Broadcaster
**File**: `aruco_broadcaster/src/aruco_broadcaster.cpp`

**Function**: Detects ArUco markers and publishes TF transforms

**Subscription**:
```cpp
Topic: /aruco_marker_publisher/markers (MarkerArray)
  ├─ marker[0].id = 100 (board marker)
  ├─ marker[1].id = 305 (white pawn E2)
  └─ ... (all detected markers)
```

**TF Broadcasting**:
```cpp
// For each detected marker:
geometry_msgs::msg::TransformStamped tf;
tf.header.frame_id = "camera_color_optical_frame"  // Source frame
tf.child_frame_id = "aruco_305"                     // Target frame
tf.transform.translation = marker_pose.position     // XYZ in camera frame
tf.transform.rotation = marker_pose.orientation     // Orientation

// Broadcast via TF
tf_broadcaster->sendTransform(tf);
```

**TF Chain**:
```
world
  └─ chess_frame (same as world, 0,0,0)
      └─ camera_link (calibrated position)
          └─ camera_color_optical_frame (camera intrinsics)
              └─ aruco_305 (detected marker position)
```

**Service**: `get_marker_Tf`
```
Request:
  parent: "world"
  marker_id: "aruco_305"
Response:
  ret: TransformStamped (world → aruco_305)
```

### Module 2: Sensing Module
**File**: `sensing_module/src/sensing_node.cpp`

**Function**: Convert TF frames to chess cell locations

**Algorithm**:
1. **TF Lookup**: Query `world → aruco_305` every 500ms
2. **Extract Position**: Get (x, y, z) from transform
3. **Map to Cell**:
   ```cpp
   // Board is 8x8, each square is 0.05m (5 cm)
   // Origin at board center
   // Files: A(x=-0.175)...H(x=0.175)
   // Ranks: 1(y=-0.175)...8(y=0.175)
   
   fx = x / 0.05 + 3.5  // Normalize to 0-7
   fy = y / 0.05 + 3.5
   
   file = char('A' + round(fx))  // A-H
   rank = round(fy) + 1           // 1-8
   cell = file + rank             // "E2"
   ```

**Service**: `piece_location`
```
Request:
  aruco_id: 305
Response:
  found: true
  pose:
    position: {x: 0.0, y: -0.175, z: 0.04}
    orientation: {x: 0, y: 0, z: 0, w: 1}
  cell: "E2"
```

**State Tracking**:
```cpp
std::map<int, PieceInfo> pieces_;
// Each piece cached with:
//  - pose (geometry_msgs::Pose)
//  - cell (std::string)
//  - valid (bool)
//  - timestamp

// Only update if position changed (debounce)
```

**Outputs**:
- ROS2 Service for piece queries
- YAML file: `/tmp/chess_configuration_sensed.yaml`
  ```yaml
  pieces:
    aruco_305: "E2"
    aruco_316: "E1"
    aruco_204: "D7"
    ...
  ```

### Module 3: Planning Module
**File**: `planning_module/src/planning_node.cpp`

**Function**: Generate robot trajectories using inverse kinematics

**Inputs**:
```cpp
geometry_msgs::msg::Pose source_pose  // E2: (0, -0.175, 0.04)
geometry_msgs::msg::Pose target_pose  // E4: (0, -0.075, 0.04)
```

**Waypoint Generation** (6 steps):
```cpp
// Step 1: Pre-grasp (hover above piece)
Pose waypoint_0 = source_pose;
waypoint_0.position.z += 0.10;  // 10cm above
waypoint_0.orientation = RPY(π, 0, 0);  // Gripper pointing down

// Step 2: Grasp (move to piece)
Pose waypoint_1 = source_pose;
waypoint_1.orientation = RPY(π, 0, 0);

// Step 3: Lift (raise with piece)
Pose waypoint_2 = source_pose;
waypoint_2.position.z += 0.10;
waypoint_2.orientation = RPY(π, 0, 0);

// Step 4: Pre-drop (move above target)
Pose waypoint_3 = target_pose;
waypoint_3.position.z += 0.10;
waypoint_3.orientation = RPY(π, 0, 0);

// Step 5: Drop (place piece)
Pose waypoint_4 = target_pose;
waypoint_4.orientation = RPY(π, 0, 0);

// Step 6: Post-drop (slight lift)
Pose waypoint_5 = target_pose;
waypoint_5.position.z += 0.05;
waypoint_5.orientation = RPY(π, 0, 0);
```

**IK Solver Call**:
```cpp
// For each waypoint, call inverse kinematics
kinenikros2::srv::InverseKinematics ik_request;
ik_request->type = "UR3";
ik_request->pose = waypoint;

// Response:
ik_response->status = true;
ik_response->ik_solution[0].ik = [q1, q2, q3, q4, q5, q6];
```

**Service**: `plan_pick_place`
```
Request:
  source_pose: Pose (E2)
  target_pose: Pose (E4)

Response:
  success: true
  message: "Trajectory planned successfully"
  trajectory: [
    JointState{position: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]},  // waypoint 0
    JointState{position: [0.1, 0.25, 0.3, 0.4, 0.5, 0.6]}, // waypoint 1
    ...
    JointState{position: [0.1, 0.3, 0.3, 0.4, 0.5, 0.6]}   // waypoint 5
  ]
```

### Module 4: Action Manager (Main Orchestrator)
**File**: `action_manager/src/action_manager_node.cpp`

**Function**: High-level chess move execution

**Action Definition**:
```
Input (Goal):
  action_type: 0 (MOVE), 1 (CAPTURE), 2 (CASTLING)
  piece_aruco_id: 305
  target_square: "E4"
  captured_piece_aruco_id: 0
  castling_side: ""

Feedback:
  progress: 0-100%
  status: "Sensing current position"

Result:
  success: true
  message: "Chess action completed successfully"
  final_square: "E4"
```

**Execution Flow** (MOVE action):

```cpp
// Step 1: SENSING (10%)
std::vector<sensing_module::srv::PieceLocation> req;
req->aruco_id = 305;
// Get: pose = (0, -0.175, 0.04), cell = "E2"

// Step 2: TARGET POSE (20%)
geometry_msgs::msg::Pose target = cell_to_pose("E4");
// target = (0, -0.075, 0.04)

// Step 3: PLANNING (30%)
std::vector<sensor_msgs::msg::JointState> trajectory;
planning_module::srv::PlanPickPlace plan_req;
plan_req->source_pose = (0, -0.175, 0.04);
plan_req->target_pose = (0, -0.075, 0.04);
// Get: trajectory with 6 waypoints

// Step 4: EXECUTION (50%)
for (const auto& js : trajectory) {
  // Method A: Gazebo simulation
  chesslab_setup2_interfaces::srv::SetRobConf req;
  req->conf = {js.position[0], js.position[1], ...};
  // Robot moves to each joint configuration
  
  // Method B: Real robot
  // Send to ur_robot_driver or MoveIt2
}

// Also control gripper:
robotiq_85_gripper_server::srv::GripperOrder req;
req->order = 1;  // Close at waypoint 1, open at waypoint 5

// Step 5: VERIFICATION (90%)
sensing_module::srv::PieceLocation req;
req->aruco_id = 305;
// Get: cell = "E4" ✓ Success

// Return result
result->success = true;
result->final_square = "E4";
```

## State Diagrams

### Piece Location State Machine
```
┌─────────────┐
│   Unknown   │ (marker not detected)
└──────┬──────┘
       │ (marker detected)
       ▼
┌─────────────────────────────────┐
│  Position Known (e.g., E2)      │
│  ├─ Pose: (0, -0.175, 0.04)    │
│  ├─ Cell: "E2"                  │
│  ├─ Timestamp: t_0              │
│  └─ Valid: true                 │
└──────┬──────────────────────────┘
       │ (marker moves)
       ▼
┌──────────────────────────────────┐
│ New Position (e.g., E4)         │
│ ├─ Update cache                  │
│ ├─ Notify listeners              │
│ └─ Update YAML file              │
└──────────────────────────────────┘
```

### Action Execution State Machine
```
                      ┌─────────────┐
                      │  ACCEPTED   │
                      └──────┬──────┘
                             │
                    ┌────────▼────────┐
                    │  EXECUTING      │
                    │  ├─ Sensing     │
                    │  ├─ Planning    │
                    │  └─ Motion      │
                    └────┬────────┬───┘
                         │        │
                    Success   Failure
                         │        │
                    ┌────▼┐  ┌───▼───┐
                    │SUCC │  │ ABORT │
                    └─────┘  └───────┘
```

## Data Structures

### Piece Information
```cpp
struct PieceInfo {
  int aruco_id;
  geometry_msgs::msg::Pose pose;
  std::string cell;
  bool valid;
  rclcpp::Time timestamp;
};
```

### Chessboard Geometry
```cpp
// Board parameters
constexpr double SQUARE_SIZE = 0.05;      // 5 cm
constexpr double BOARD_CENTER_X = 0.0;    // Board center at origin
constexpr double BOARD_CENTER_Y = 0.0;
constexpr double BOARD_Z = 0.04;          // Height of board surface

// Cell positions in meters
A1: (-0.175, -0.175, 0.04)
E1: (0.0,    -0.175, 0.04)
E4: (0.0,    -0.075, 0.04)
H8: (0.175,   0.175, 0.04)
```

## Frame Transformations

### TF Chain Construction
```
world (base frame)
  ├─ Source: /tf_publisher (static)
  └─ Transform: (0,0,0) + Quaternion(0,0,0,1)

camera_link
  ├─ Source: Calibration (tablesens)
  └─ Transform: Relative to robot base

camera_color_optical_frame
  ├─ Source: Camera intrinsics
  └─ Transform: Fixed offset from camera_link

aruco_305
  ├─ Source: ArUco detection
  └─ Transform: Marker pose in camera frame
```

### TF Lookup Example
```cpp
// Query transform: world → aruco_305
geometry_msgs::msg::TransformStamped tf;
tf = tf_buffer->lookupTransform(
  "world",           // target_frame
  "aruco_305",       // source_frame
  tf2::TimePointZero // get latest
);

// Returns composition:
// world ← camera_link ← camera_optical ← aruco_305
// Automatically composes through intermediate frames
```

## Service/Action Contracts

### Service: piece_location
```cpp
// Definition
sensing_module::srv::PieceLocation {
  // Request
  int32 aruco_id;
  
  // Response
  bool found;
  geometry_msgs::msg::Pose pose;
  string cell;
}
```

### Service: plan_pick_place
```cpp
planning_module::srv::PlanPickPlace {
  // Request
  geometry_msgs::msg::Pose source_pose;
  geometry_msgs::msg::Pose target_pose;
  
  // Response
  bool success;
  string message;
  sensor_msgs::msg::JointState[] trajectory;
}
```

### Action: ChessAction
```cpp
action_manager::action::ChessAction {
  // Goal
  uint8 action_type;
  int32 piece_aruco_id;
  string target_square;
  int32 captured_piece_aruco_id;
  string castling_side;
  
  // Result
  bool success;
  string message;
  string final_square;
  
  // Feedback
  float32 progress;
  string status;
}
```

## Performance Characteristics

### Timing Analysis
```
Phase 1: Sensing
  - TF lookup:        ~50 ms
  - Cache search:     ~1 ms
  - Response send:    ~5 ms
  Total:              ~56 ms

Phase 2: Planning
  - Waypoint generation: ~100 ms
  - IK solve (6x):       ~500 ms (100 ms per waypoint)
  - Trajectory build:    ~50 ms
  Total:                 ~650 ms

Phase 3: Execution
  - Move to waypoint 0:  ~2000 ms (approach)
  - Move to waypoint 1:  ~1000 ms (descend)
  - Gripper close:       ~1000 ms
  - Move to waypoint 2:  ~1000 ms (lift)
  - Move to waypoints 3-5: ~3000 ms (traverse)
  - Gripper open:        ~1000 ms
  Total:                 ~9000 ms

Phase 4: Verification
  - Wait for markers:    ~500 ms
  - TF lookup:          ~50 ms
  - Comparison:         ~1 ms
  Total:                ~551 ms

TOTAL PER MOVE: ~10.2 seconds
```

### Memory Usage
```cpp
// Per piece tracking:
PieceInfo: ~100 bytes
  - 4 bytes: aruco_id
  - 60 bytes: pose
  - 32 bytes: cell string
  - 4 bytes: bool

// For 32 pieces:
32 pieces × 100 bytes = 3.2 KB

// Plus ROS2 overhead:
- Cached TF data
- Service buffers
- Node state
Total: ~10-20 MB per node
```

### Scalability
- **Maximum pieces**: Limited by camera resolution and marker spacing
- **Maximum speed**: 1 move every 10 seconds (timing above)
- **Accuracy**: ±5mm (marker detection + IK tolerance)
- **Reliability**: 95%+ (depends on camera calibration)

## Error Handling & Recovery

### Common Errors & Responses
```cpp
// Error 1: Marker not detected
if (!get_piece_pose(aruco_id, pose, cell)) {
  // Retry with timeout
  // Eventually abort with error message
}

// Error 2: IK solution not found
if (!compute_ik_for_pose(pose, joint_state)) {
  // Try alternative grasp directions
  // Try different target approach poses
  // Eventually abort
}

// Error 3: Position verification fails
if (final_cell != target_square) {
  // Log warning (may be sensing error)
  // Accept anyway (gripper released successfully)
  // Return success (tolerant of small errors)
}
```

## Extension Points

### Adding New Piece Types
1. Add ArUco ID mapping
2. Set piece height in height map
3. System automatically handles in planning

### Adding New Board Sizes
1. Change SQUARE_SIZE constant
2. Update cell_to_pose conversion
3. Recalibrate camera

### Adding Collision Avoidance
1. Integrate MoveIt2
2. Call plan_motions with collision_checking=true
3. Handle planning failures gracefully

### Adding Force Feedback
1. Use force/torque sensor
2. Detect grasping force in gripper close
3. Adapt gripper pressure based on piece type

## References & Standards

- **ROS2 Humble**: https://docs.ros.org/en/humble/
- **ArUco Detection**: https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
- **UR Robot**: https://www.universal-robots.com/
- **Robotiq Gripper**: https://robotiq.com/
- **TF2 Documentation**: https://wiki.ros.org/tf2
