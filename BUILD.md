# Build and Installation Instructions

## System Requirements

### Operating System
- Ubuntu 22.04 LTS (recommended)
- Ubuntu 20.04 LTS (may work with ROS2 Foxy)

### Hardware (for real system)
- UR3 or UR3e robot arm
- Robotiq 85 gripper
- RealSense D455 camera
- Chess board with ArUco markers (provided)
- Host PC with ethernet connection to robot

### Software Dependencies
```bash
# Core ROS2 and build tools
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    build-essential \
    git \
    wget

# ROS2 packages
sudo apt install -y \
    ros-humble-gazebo-ros \
    ros-humble-tf2-ros \
    ros-humble-rclcpp \
    ros-humble-rclcpp-action \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-opencv-apps \
    ros-humble-cv-bridge \
    ros-humble-image-transport

# Optional visualization tools
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-plotjuggler
```

## Repository Setup

### 1. Clone the Main Repository
```bash
cd ~
git clone https://github.com/Ale-Tac/colcon_FW_sec.git
cd colcon_FW_sec
```

### 2. Install External Dependencies
The project requires several external packages. These should be in `src/` directory:

```bash
cd src

# Required packages (these must be available)
# - aruco_ros (ArUco marker detection)
# - chesslab_setup2 (robot + scene setup)
# - chesslab_setup2_interfaces (service definitions)
# - kinenikros2 (UR kinematics)
# - robotiq_85_gripper (hardware driver)
# - robotiq_85_gripper_server (ROS2 interface)
# - tablesens (camera calibration)

# If not present, link from your source location:
ln -s /path/to/external/aruco_ros aruco_ros
ln -s /path/to/external/chesslab_setup2 chesslab_setup2
# ... etc
```

**If external packages are not available:**
- You can still build `final-work` modules in isolation
- They will require the external packages to run

## Building

### Build Everything
```bash
cd ~/colcon_FW_sec
colcon build
```

### Build Only final-work Packages
```bash
colcon build --packages-select \
    aruco_broadcaster \
    sensing_module \
    planning_module \
    action_manager
```

### Build Specific Package
```bash
colcon build --packages-select action_manager
```

### Clean Build
```bash
rm -rf build install
colcon build
```

### Build with Additional Output
```bash
# Verbose output
colcon build --event-handlers console_direct+

# Generate compilation database
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## Build Output

After successful build, you should see:

```
Finished `colcon build` at [timestamp]
Summary: X packages finished [Y/Z left]
```

Build artifacts are in:
- `build/` - CMake build directories
- `install/` - Installed executables and libraries
- `log/` - Build logs

## Environment Setup

### Source ROS2 and Project
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Source your colcon workspace
cd ~/colcon_FW_sec
source install/setup.bash
```

### Create Convenience Script
```bash
# Create ~/.bashrc.local
cat > ~/.bashrc.local << 'EOF'
# ROS2 Chess Robot Setup
source /opt/ros/humble/setup.bash
source ~/colcon_FW_sec/install/setup.bash
export ROS_DOMAIN_ID=0
EOF

# Add to ~/.bashrc
echo "source ~/.bashrc.local" >> ~/.bashrc
source ~/.bashrc
```

## Verify Installation

### Check Packages
```bash
# List all available packages
ros2 pkg list | grep -E "(action_manager|sensing|planning|aruco)"

# Show package details
ros2 pkg list action_manager
```

### Check Executables
```bash
# List all executables in a package
ros2 pkg executables action_manager

# Show available nodes
ros2 node list  # (after launching)
```

### Check Build Artifacts
```bash
# Verify installations
ls -la install/action_manager/lib/
ls -la install/action_manager/share/

# Check launch files
ls -la install/final_work/share/final_work/launch/
```

## Troubleshooting Build Issues

### Issue: "Package not found"
```bash
# Ensure workspace is sourced
source install/setup.bash

# Rebuild specific package
colcon build --packages-select action_manager --force-cmake-configure
```

### Issue: "CMake version too old"
```bash
# Update CMake
sudo apt install -y cmake

# Or install from source
wget https://github.com/Kitware/CMake/releases/download/v3.24.0/cmake-3.24.0.tar.gz
```

### Issue: "Dependency not found"
```bash
# Check dependencies are installed
apt search ros-humble-rclcpp

# Install missing ROS2 package
sudo apt install -y ros-humble-rclcpp-action
```

### Issue: Compilation errors
```bash
# Clean build with verbose output
rm -rf build install
colcon build --event-handlers console_direct+

# Check error messages carefully for:
# - Missing headers (add to CMakeLists.txt)
# - Undefined references (check linking)
# - Type mismatches (check header includes)
```

## Docker Build (Optional)

If you have Docker installed, you can build in a container:

```dockerfile
FROM osrf/ros:humble-desktop

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    build-essential git

WORKDIR /ws
COPY . /ws/src

RUN . /opt/ros/humble/setup.sh && \
    colcon build

ENTRYPOINT ["/bin/bash"]
```

Build with:
```bash
docker build -t chess-robot .
docker run -it chess-robot
```

## Next Steps

After successful build:

1. **Test in Simulation**
   ```bash
   ros2 launch final_work chess_system.launch.py mode:=sim
   ```

2. **Test Perception Only**
   ```bash
   ros2 launch final_work chess_perception.launch.py
   ```

3. **Run Examples**
   ```bash
   # See examples/ directory for test scripts
   cat src/final-work/examples/example_1_startup_simulation.sh
   ```

4. **Run Validation**
   ```bash
   bash src/final-work/examples/run_validation.sh
   ```

## Build Configuration Reference

### CMakeLists.txt Structure
Each package has a CMakeLists.txt with:
- Find dependencies
- Build executables
- Generate interfaces (services/actions)
- Install artifacts

### ROS2 Build System
- Uses `ament_cmake` (CMake-based)
- Services: `.srv` files in `srv/` directory
- Actions: `.action` files in `action/` directory
- Generated from IDL → C++ includes

### Package Dependencies
Defined in `package.xml`:
```xml
<depend>rclcpp</depend>
<depend>geometry_msgs</depend>
<build_depend>rosidl_default_generators</build_depend>
```

## Performance Tips

### Faster Builds
```bash
# Parallel build (use all CPU cores)
colcon build -j $(nproc)

# Skip package with no changes
colcon build --skip-until action_manager

# Build only what changed
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### Smaller Build Size
```bash
# Release build (optimized, no debug symbols)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Strip binaries
strip install/*/lib/*/*.so
```

## Continuous Integration

For CI/CD pipelines:

```bash
#!/bin/bash
set -e  # Exit on error

source /opt/ros/humble/setup.bash

# Build
colcon build --event-handlers console_direct+

# Test (if tests exist)
colcon test --event-handlers console_direct+

# Check result
if [ $? -eq 0 ]; then
    echo "Build successful"
else
    echo "Build failed"
    exit 1
fi
```

## Advanced Topics

### Custom Build Options
```bash
colcon build \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    --mixin comp
```

### Build for Real Hardware
```bash
# Cross-compile for robot platform (if needed)
colcon build --merge-install \
    --cmake-args \
        -DCMAKE_TOOLCHAIN_FILE=<toolchain> \
        -DCMAKE_PREFIX_PATH=/opt/ros/humble
```

### Generate Documentation
```bash
# Build Doxygen documentation
colcon build --event-handlers cmake_test_event_handler

# Find generated docs
find build -name "html" -o -name "latex"
```

## Support

If build fails:
1. Check error messages carefully
2. Ensure all dependencies are installed
3. Try clean build: `rm -rf build install && colcon build`
4. Check ROS2 installation: `ros2 doctor`
5. Review CMakeLists.txt and package.xml files
6. Post to ROS2 Discourse: https://discourse.ros.org/
