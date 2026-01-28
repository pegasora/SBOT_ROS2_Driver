# StandardBot ROS2 Driver

ROS2 workspace for controlling StandardBot robots in a manufacturing line.

## Prerequisites

### Ubuntu 22.04 (Recommended for Students)

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

   Create a virtual environment and install dependencies:
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
   
   **Note:** We don't keep the venv activated. Instead, `setup_workspace.sh` will overlay the venv packages into system Python's path.


5. **Setup ROS2 Environment**

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

```bash
# Setup environment (do this FIRST!)
source setup_workspace.sh

# Build the workspace
just build

# Launch the robot drivers
just launch
```

### Manual Commands

```bash
# Setup environment (do this FIRST!)
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

### Building

```bash
# list available commands
just list

# Build workspace
just build

# Clean build artifacts
just clean

# Clean and rebuild
just rebuild

# Build with verbose output
just build-verbose
```

### Running

```bash
# Launch the demo
just launch

# Launch with arguments
just launch robot_ip:=192.168.1.100

# Check ROS2 environment
just check-env

# List installed packages
just list-packages

# List available nodes
just list-nodes
```

### Testing

```bash
# Run tests
just test
```

### Development

```bash
# Format code
just format

# Lint code
just lint

# Install dependencies
just install-deps
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

Make sure venv is created and packages installed:
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e .
deactivate
```

Then source the workspace:
```bash
source setup_workspace.sh
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
- Add camera server node

## Contributors

- Course Staff
- Students
