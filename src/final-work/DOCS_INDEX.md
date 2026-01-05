# Chess Robot System - Debug & Documentation Index

**Quick Navigation Guide for the Chess-Playing Robot Project**

---

## 🚨 I Have a Problem - Where Do I Start?

### My Issue: Piece location is wrong (e.g., "A4" instead of "E1")
→ **Start here**: Read the COORDINATE_FIX_GUIDE.md in this directory
→ **Quick fixes**: See QUICK_FIXES.md (copy-paste solutions)
→ **Detailed help**: See SENSING_COORDINATE_DEBUG.md

### I want to understand the system architecture
→ **Start here**: See COMPLETE_SYSTEM_SUMMARY.md
→ **Then read**: action_manager/include/action_manager/action_manager_node.hpp

### I want to build and run the system
→ **Quick start**: See BUILD_AND_RUN.md

---

## 📚 Documentation Files Available

### Emergency/Debugging
| File | Purpose |
|------|---------|
| COORDINATE_FIX_GUIDE.md | Overview of coordinate bug and solutions |
| QUICK_FIXES.md | Copy-paste fixes for coordinate issue |
| SENSING_COORDINATE_DEBUG.md | Detailed technical analysis |
| SENSING_COORDINATE_ALTERNATIVES.hpp | 6 different formula options |

### System Overview
| File | Purpose |
|------|---------|
| COMPLETE_SYSTEM_SUMMARY.md | Full architecture and status |
| BUILD_AND_RUN.md | Build instructions and testing |

---

## 🎯 Quick Access

### For Sensing Coordinate Bug
1. Run: `bash run_sensing_diagnostic.sh`
2. Read: `QUICK_FIXES.md`
3. Apply fix to: `final-work/sensing_module/src/sensing_node.cpp`

### For Understanding Everything
1. Read: `COMPLETE_SYSTEM_SUMMARY.md`
2. Review code: `action_manager/include/action_manager/action_manager_node.hpp`

### For Building
1. Read: `BUILD_AND_RUN.md`
2. Follow build steps
3. Run: `colcon build && source install/setup.bash`

---

**Status**: 🟡 System 90% complete - fixing coordinate issue  
**Estimated Fix Time**: 15-30 minutes
