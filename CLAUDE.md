# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is a ROS1 (Catkin) package for robot teleoperation and imitation learning using Gello haptic devices, supporting multiple robot platforms including Universal Robots (UR5e), COBOTTA, and Franka FR3. The package integrates teleoperation agents, camera systems, and ACT (Action Chunking with Transformers) policy learning.

## Build System and Environment

This package uses **ROS1 Catkin** build system (NOT ROS2):
- Uses traditional `package.xml` with format="2" and `catkin` buildtool_depend
- Python setup handled through `catkin_python_setup()` in CMakeLists.txt
- Install dependencies: `pip install -r requirements.txt`

### Build Commands
```bash
# Build the package (from workspace root)
catkin build gello_ros

# Or build entire workspace
catkin build

# Source the workspace
source devel/setup.bash
```

### Python Dependencies
Core dependencies include: PyTorch, OpenCV, pyrealsense2, dynamixel-sdk, ur-rtde, xarm-python-sdk, h5py, einops.

## Core Architecture

### Agent System
The package implements a modular agent architecture:

- **Base Agent Protocol** (`src/gello_ros/agents/agent.py`): Defines `Agent.act(obs) -> action` interface
- **Teleoperation Agents**:
  - `GelloAgent`: Haptic device teleoperation
  - `TouchAgent`: Touch-based control with force feedback
  - `SpacemouseAgent`: 3D mouse control
  - `QuestAgent`: VR controller teleoperation
- **Learning Agent**:
  - `ACTAgent`: Executes trained ACT policies for autonomous operation

### Environment System
`RobotEnv` (`src/gello_ros/env.py`) provides unified robot control interface:
- Supports both "joint" and "cartesian" control modes
- Integrates multiple camera streams
- Configurable control rates (default 100Hz, configurable up to 500Hz)

### Robot Abstraction Layer
Multiple robot implementations in `src/gello_ros/robots/`:
- `panda.py`: Franka FR3 robot control
- `ur.py`: Universal Robots UR5e control
- Robot controllers support: joint_trajectory, cartesian_compliance, cartesian_impedance

### Configuration System
YAML-based configuration in `config/` directory:
- `common.yaml`: General operation parameters (control_hz, camera settings, task config)
- Robot-specific configs: `ur5e.yaml`, `fr3.yaml`, `cobotta.yaml`
- Device configs: `gello.yaml`, `touch.yaml`

## Common Development Commands

### Gello Device Calibration
```bash
# Get offset for UR5e
python3 scripts/gello_get_offset.py --start-joints -1.57 -1.57 -1.57 -1.57 1.57 0 --joint-signs 1 1 -1 1 1 1 --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISUQE-if00-port0

# Get offset for COBOTTA  
python3 scripts/gello_get_offset.py --start-joints 0 0 1.57 0 1.57 0 --joint-signs 1 1 -1 1 -1 1 --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT88YXAT-if00-port0

# Check available serial ports
ls /dev/serial/by-id/
```

### Launch System
```bash
# Run teleoperation agent
roslaunch gello_ros run_agent.launch agent_type:=touch robot_config:=config/ur5e.yaml controller_type:=cartesian_impedance_controller

# Run agent with camera
roslaunch gello_ros run_agent_and_cam.launch

# For COBOTTA robot
roslaunch gello_ros run_agent_for_cobotta.launch

# Test camera systems
roslaunch gello_ros test_camera.launch
roslaunch gello_ros run_cam_realsense.launch
roslaunch gello_ros run_cam_uvc.launch
```

### ACT Policy Training and Evaluation
```bash
# Train ACT policy
cd scripts/
python train.py

# Run trained policy
python run_agent_node.py  # with agent_type="act" in config
```

### Camera Configuration
Camera vendor/product ID detection:
```bash
# Check camera device info
udevadm info --name=/dev/video0

# List USB cameras
lsusb

# Check camera formats
v4l2-ctl --device /dev/video0 --list-formats-ext
```

## Key Configuration Parameters

### Control Modes
- `control_mode`: "joint" or "cartesian" 
- `controller_type`: "joint_trajectory_controller", "cartesian_compliance_controller", or "cartesian_impedance_controller"
- `teleoperation_mode`: "unilateral" or "bilateral" (for force feedback)

### Camera System
- Supports RealSense and UVC cameras
- Configurable resolution, FPS in `common.yaml`
- Multiple camera views: "base", "side", "bottom"

### Episode Recording
- `save_episode`: Enable/disable data collection
- Episodes saved to HDF5 format for ACT training
- Configurable episode length and number in `common.yaml`

## Hardware Notes

### Serial Communication
- Gello devices use FTDI USB-Serial converters
- Recommended baud rate: 2M (4M causes instability)
- Device identification by USB serial ID for consistent connection

### Force Feedback
- Touch agent supports bilateral teleoperation with force feedback
- Force scaling and limits configurable in `common.yaml`
- Leptrino force sensor integration

### Robot-Specific Notes
- UR5e: Uses `ur-rtde` for real-time communication
- COBOTTA: Integration with DENSO robot controller
- FR3: Franka real-time control interface
- All robots support impedance/compliance control modes