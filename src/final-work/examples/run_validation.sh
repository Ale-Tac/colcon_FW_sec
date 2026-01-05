#!/bin/bash

# Master Test Script for Chess Robot System
# Validates all components work correctly

echo "================================================"
echo "Chess Robot System - Validation Test Suite"
echo "================================================"
echo ""

# Color codes for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test counter
TESTS_PASSED=0
TESTS_FAILED=0

# Function to run a test
run_test() {
    local test_name="$1"
    local test_command="$2"
    local expected_success="$3"
    
    echo -n "Testing: $test_name ... "
    
    if eval "$test_command" > /dev/null 2>&1; then
        if [ "$expected_success" = "true" ]; then
            echo -e "${GREEN}PASS${NC}"
            ((TESTS_PASSED++))
            return 0
        else
            echo -e "${RED}FAIL (expected to fail)${NC}"
            ((TESTS_FAILED++))
            return 1
        fi
    else
        if [ "$expected_success" = "false" ]; then
            echo -e "${GREEN}PASS (correctly failed)${NC}"
            ((TESTS_PASSED++))
            return 0
        else
            echo -e "${RED}FAIL${NC}"
            ((TESTS_FAILED++))
            return 1
        fi
    fi
}

echo "Pre-flight Checks:"
echo "=================="
echo ""

# Check 1: ROS2 installation
echo -n "1. ROS2 installed ... "
if command -v ros2 &> /dev/null; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}NOT FOUND${NC}"
    echo "   Install ROS2 Humble:"
    echo "   https://docs.ros.org/en/humble/Installation.html"
fi

# Check 2: Built packages
echo -n "2. Package 'action_manager' built ... "
if [ -d "install/action_manager" ]; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}NOT FOUND${NC}"
    echo "   Run: colcon build --packages-select action_manager"
fi

# Check 3: Launch files
echo -n "3. Launch files exist ... "
if [ -f "src/final-work/launch/chess_system.launch.py" ]; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}NOT FOUND${NC}"
fi

echo ""
echo "Package Structure Validation:"
echo "============================="
echo ""

# Check package.xml files
for pkg in action_manager sensing_module planning_module aruco_broadcaster; do
    echo -n "Checking $pkg ... "
    if [ -f "src/final-work/$pkg/package.xml" ]; then
        echo -e "${GREEN}OK${NC}"
    else
        echo -e "${RED}MISSING package.xml${NC}"
    fi
done

echo ""
echo "Source File Validation:"
echo "======================"
echo ""

# Check C++ source files
for src in \
    "src/final-work/action_manager/src/action_manager_node.cpp" \
    "src/final-work/action_manager/include/action_manager/action_manager_node.hpp" \
    "src/final-work/planning_module/src/planning_node.cpp" \
    "src/final-work/sensing_module/src/sensing_node.cpp" \
    "src/final-work/aruco_broadcaster/src/aruco_broadcaster.cpp"; do
    
    if [ -f "$src" ]; then
        echo -e "${GREEN}✓${NC} $src"
    else
        echo -e "${RED}✗${NC} $src (missing)"
    fi
done

echo ""
echo "Service Definition Validation:"
echo "=============================="
echo ""

for srv in \
    "sensing_module/srv/PieceLocation.srv" \
    "planning_module/srv/PlanPickPlace.srv" \
    "aruco_broadcaster/srv/GetMarkerTf.srv"; do
    
    if [ -f "src/final-work/$srv" ]; then
        echo -e "${GREEN}✓${NC} $srv"
    else
        echo -e "${RED}✗${NC} $srv (missing)"
    fi
done

echo ""
echo "Action Definition Validation:"
echo "============================="
echo ""

if [ -f "src/final-work/action_manager/action/ChessAction.action" ]; then
    echo -e "${GREEN}✓${NC} ChessAction.action"
else
    echo -e "${RED}✗${NC} ChessAction.action (missing)"
fi

echo ""
echo "================================================"
echo "Summary:"
echo "================================================"
echo ""
echo "Total tests: $((TESTS_PASSED + TESTS_FAILED))"
echo -e "Passed: ${GREEN}$TESTS_PASSED${NC}"
echo -e "Failed: ${RED}$TESTS_FAILED${NC}"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All checks passed!${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Install ROS2 Humble (if not already installed)"
    echo "  2. Build packages: colcon build"
    echo "  3. Source environment: source install/setup.bash"
    echo "  4. Launch simulation: ros2 launch final_work chess_system.launch.py mode:=sim"
    echo "  5. In another terminal, test: see examples/ directory"
    exit 0
else
    echo -e "${RED}✗ Some checks failed. See above for details.${NC}"
    exit 1
fi
