# StandardBot ROS2 Driver

ROS2 workspace for controlling StandardBot robots in a manufacturing line.

## Prerequisites

### Ubuntu 24.04 (Recommended for Students)

1. **Install ROS2 Jazzy**
 [Link](https://github.com/pegasora/UofI_AdvRobotics)

2. **Install Development Tools**

    You have likely already installed these tools, but if not, you can install them with the following commands:
   ```bash
   # Install colcon build tool
   sudo apt install python3-colcon-common-extensions

   # Install pip and Python tools
   sudo apt install python3-pip python3-venv

   # Install just (optional but recommended)
   sudo apt install just
   ```

3. **Install ROS2 Vision Packages (Required for Camera Support)**

   ```bash
   # Install cv_bridge and related vision packages
   sudo apt install ros-jazzy-cv-bridge ros-jazzy-vision-opencv ros-jazzy-image-transport
   
   # Install common message packages
   sudo apt install ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-trajectory-msgs ros-jazzy-action-msgs
   ```

4. **Install Python Dependencies**

   **Option A: Using just (recommended)**
   ```bash
   just setup
   ```
   
   **Option B: Manual setup**
   ```bash
   # Create virtual environment
   python3 -m venv .venv
   
   # Activate it
   source .venv/bin/activate
   
   # Install dependencies
   pip install -e .
   
   # Deactivate (we don't keep it activated)
   deactivate
   ```
   
   **How it works:** The venv is NOT activated during runtime. Instead, `setup_workspace.sh` overlays the venv packages into system Python's PYTHONPATH so ROS2 nodes can import them.


5. **Setup ROS2 Environment**
   You have likely already done this, if so, skip to step 6.

   Add to your `~/.bashrc`:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

   Then reload:
   ```bash
   source ~/.bashrc
   ```

6. **Setup Workspace Script**

   Make the setup script executable:
   ```bash
   chmod +x setup_workspace.sh
   ```
   
   Now you can run `source setup_workspace.sh` to source ROS2, overlay venv packages, and source workspace.

## Quick Start

### Using Just (Recommended)
You only need to run the setup command once. For the build command, you only need to run it once per changes made.
```bash
# First time setup (only once)
just setup

# Source the workspace (every new terminal)
source setup_workspace.sh

# Build the workspace
just build

# Launch the robot drivers
just launch
```

### Manual Commands (without just)

```bash
# First time setup
python3 -m venv .venv
source .venv/bin/activate
pip install -e .
deactivate

# Every session
source setup_workspace.sh

# Build
colcon build --symlink-install

# Launch
ros2 launch standard_bot_bringup demo.launch.py
```

## Workspace Packages

- **action_interfaces** - Custom ROS2 action definitions for robot control
  - `SetCartPos.action` - Set cartesian position
  - `SetJointPos.action` - Set joint positions
  - `OnRobotGripper.action` - Control on-robot gripper

- **sbot_interfaces** - Custom messages and services
  - `GripperStatus.msg` - Gripper state information
  - `RobotStatus.msg` - Robot state information
  - `SensorStatus.msg` - Sensor readings
  - `GetRobotStatus.srv` - Service to query robot status

- **sb_controller** - Robot controller nodes
  - Status monitoring nodes
  - Action servers for robot control
  - Client nodes for testing

- **standard_bot_bringup** - Launch files and configuration
  - `demo.launch.py` - Main launch file

## Common Commands
To list all commands in the justfile, you can simply:
```bash
just 
```
### Setup (First Time)

```bash
# Setup venv and install dependencies (once)
just setup

# Activate environment (every new terminal)
source setup_workspace.sh  # Run this
```

### Building

```bash
# Build workspace
just build

# Clean build artifacts
just clean

# Clean and rebuild
just rebuild
```

### Running

```bash
# Launch the demo
just launch

# Launch with arguments
just launch robot_ip:=xxx.xxx.xxx.xxx

# Check ROS2 environment
just check-env
```

### Development

```bash
# Setup environment
just setup  # First time only
```

## Troubleshooting

### "ros2: command not found"

Make sure you've sourced the ROS2 setup:
```bash
source /opt/ros/jazzy/setup.bash
```

### "package 'X' not found"

Make sure you've built the workspace and sourced it:
```bash
colcon build --symlink-install
source install/setup.bash
```

### Build Errors

Try cleaning and rebuilding:
```bash
just rebuild
```

### Python Module Not Found (e.g., standardbots)

Run the setup command:
```bash
just setup
```

Then source the workspace:
```bash
source setup_workspace.sh
```

If the problem persists, manually verify:
```bash
ls .venv/lib/python*/site-packages/ | grep standardbots
```

## Dependencies

### System Dependencies
- ROS2 Jazzy
- Python 3.10+
- colcon

### ROS2 Package Dependencies
- ros-jazzy-cv-bridge (Camera/OpenCV bridge)
- ros-jazzy-vision-opencv (Vision utilities)
- ros-jazzy-image-transport (Image transport)
- ros-jazzy-sensor-msgs (Sensor messages including Image)
- ros-jazzy-geometry-msgs (Geometry messages)
- ros-jazzy-trajectory-msgs (Trajectory messages)
- ros-jazzy-action-msgs (Action messages)

### Python Dependencies (from pyproject.toml)
- numpy >= 2.2.5
- opencv-python >= 4.11.0.86
- pandas >= 2.2.3
- pycomm3 >= 1.2.14
- pymodbus >= 3.9.2
- standardbots == 2.20241120.1

## Project Structure

```
SBOT_ROS2_Driver/
├── src/
│   ├── action_interfaces/    # Action definitions
│   ├── sbot_interfaces/      # Messages and services
│   ├── sb_controller/        # Controller nodes
│   └── standard_bot_bringup/ # Launch files
├── pyproject.toml           # Python dependencies
├── justfile                 # Build commands
└── README.md               # This file
```

## License

TODO: 
- Add license information

## Contributors

- Dawson Burgess
- Sarah Davis (Original Standardbot ROS2 Driver)
