# Quick Start Guide - Chess Playing Robot

Get the system running in 5 minutes!

## Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble installed: https://docs.ros.org/en/humble/Installation.html
- This repository cloned and built

## 1. Build the System
```bash
cd ~/colcon_FW_sec
colcon build --packages-select \
    aruco_broadcaster \
    sensing_module \
    planning_module \
    action_manager
```

Expected output:
```
Finished `colcon build` at 2025-01-03T10:30:45...
Summary: 4 packages finished
```

## 2. Setup Environment
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 3. Launch Full System (Simulation)
```bash
ros2 launch final_work chess_system.launch.py mode:=sim ur_type:=ur3
```

You should see:
- Gazebo window opens with robot and chess board
- RViz window shows TF frames and markers
- Terminal output from each node

Wait for message:
```
[action_manager] ActionManager initialized successfully
```

## 4. Test in Another Terminal
Keep the launch running, open new terminal:

```bash
# Source again
source /opt/ros/humble/setup.bash
source ~/colcon_FW_sec/install/setup.bash

# Query a piece location
ros2 service call /piece_location sensing_module/srv/PieceLocation \
  "{aruco_id: 316}"
```

Response:
```
found: true
cell: 'E1'
```

## 5. Move a Piece
```bash
ros2 action send_goal /chess_action action_manager/action/ChessAction \
  "{
    action_type: 0,
    piece_aruco_id: 305,
    target_square: 'E4',
    captured_piece_aruco_id: 0,
    castling_side: ''
  }" \
  --feedback
```

Watch as the robot:
1. Senses the pawn location (E2)
2. Plans the trajectory
3. Closes gripper, moves, opens gripper
4. Verifies final position (E4)

You should see:
```
[action_manager] [MOVE] Step 1/5: Sensing current position
[action_manager] Piece 305 at E2
[action_manager] [MOVE] Step 2/5: Computing target pose
[action_manager] [MOVE] Step 3/5: Planning trajectory
[action_manager] [MOVE] Step 4/5: Executing motion
[action_manager] [MOVE] Step 5/5: Verifying result
[action_manager] Final position: E4
[action_manager] SUCCESS: Piece at target square
```

## Common Issues & Solutions

### "ros2: command not found"
```bash
source /opt/ros/humble/setup.bash
```

### "Package not found: action_manager"
```bash
# Ensure built and sourced
colcon build --packages-select action_manager
source install/setup.bash
```

### "Service not available"
- Check all nodes are running: `ros2 node list`
- Check services exist: `ros2 service list`
- Wait a few seconds, services may be initializing

### Gazebo doesn't open
```bash
# Try without GUI
ros2 launch final_work chess_system.launch.py mode:=sim gui:=false

# Or check Gazebo installation
gazebo --version
```

## Next Steps

1. **Learn the Architecture**: Read `src/README.md`
2. **Run Examples**: See `src/final-work/examples/`
3. **Test Capture**: Try `example_4_capture.sh`
4. **Read Build Guide**: See `BUILD.md`
5. **Explore Code**: Check `src/final-work/*/src/`

## Useful Commands

```bash
# List all nodes
ros2 node list

# List all services
ros2 service list

# List all topics
ros2 topic list

# Show node info
ros2 node info /action_manager

# Echo topic
ros2 topic echo /topic_name

# Call service
ros2 service call /service_name package/srv/Service "{param: value}"

# Send action
ros2 action send_goal /chess_action ... --feedback

# View TF tree
ros2 run tf2_tools view_frames
```

## System Architecture (Quick Reference)

```
User Terminal
     ↓
Action Manager (orchestrator)
     ↓ (calls)
┌────┬────┬────┐
│    │    │    │
▼    ▼    ▼    ▼
Sense Plan Execute Gripper
     ↓     ↓     ↓    ↓
     └─────┴─────┴────┘
          TF/Markers
             ↓
    Robot + Scene (Gazebo)
```

## File Locations

- **Launch files**: `src/final-work/launch/`
- **Source code**: `src/final-work/*/src/`
- **Headers**: `src/final-work/*/include/`
- **Services**: `src/final-work/*/srv/`
- **Actions**: `src/final-work/action_manager/action/`
- **Examples**: `src/final-work/examples/`

## Performance

- **Sensing**: 2 Hz (piece location updates)
- **Planning**: 2-5 seconds (trajectory generation)
- **Execution**: 5-10 seconds (robot motion + gripper)
- **Total per move**: 10-20 seconds

## Reality Check

This is a **learning/demo system**. For production:
- Add collision detection (MoveIt2)
- Add force feedback
- Implement chess rules validation
- Add piece shape detection
- Improve IK solver robustness
- Add comprehensive error handling

## Getting Help

1. Check terminal output for error messages
2. Run validation: `bash src/final-work/examples/run_validation.sh`
3. Review detailed docs: `src/README.md`
4. Check node source code comments
5. Post to ROS2 Discourse with error message

## Success Criteria

You'll know it's working when:
- ✓ Gazebo launches with robot
- ✓ RViz shows TF frames
- ✓ Piece location service works
- ✓ Robot moves to target square
- ✓ Sensing verifies final position

Enjoy! 🚀♟️🤖
