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

3. **Install Python Dependencies**

   **Using pip (standard method):**
   ```bash
   # Install dependencies from pyproject.toml
   pip install -e .
   
   # Or manually:
   pip install numpy>=2.2.5 opencv-python>=4.11.0.86 pandas>=2.2.3 \
               pycomm3>=1.2.14 pymodbus>=3.9.2 standardbots==2.20241120.1
   ```

   **Using uv (faster alternative):**
   
   [uv](https://github.com/astral-sh/uv) is a fast Python package installer and resolver, written in Rust.
   
   ```bash
   # Install uv
   curl -LsSf https://astral.sh/uv/install.sh | sh
   
   # Install dependencies using uv
   uv pip install -e .
   
   # Or create and sync a virtual environment
   uv venv
   source .venv/bin/activate
   uv pip sync uv.lock  # Uses the lockfile for reproducible installs
   ```
   
   **Note:** If you use uv, make sure to activate the virtual environment before running ROS2 commands.

   # IMPORTANT:
   **Note:** If you setup ROS2 to build correctly with setup.py and the xml files, you can just source the install/setup.bash file and that will work file.


4. **Setup ROS2 Environment**

   Add to your `~/.bashrc`:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

   Then reload:
   ```bash
   source ~/.bashrc
   ```

   or you will have to run this on terminal init.

## Quick Start

### Using Just (Recommended)

```bash
# Build the workspace
just build

# Source the workspace (run the printed command)
source install/setup.bash

# Launch the robot drivers
just launch

# See all available commands
just list
```

### Manual Commands

```bash
# Build
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Launch
ros2 launch standard_bot_bringup demo.launch.py
```
or call the launch file directly:
```bash
ros2 launch ./src/standard_bot_bringup/launch/demo.launch.py
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

### Python Module Not Found

Install the dependencies:
```bash
pip install -e .
```

## Dependencies

### System Dependencies
- ROS2 Jazzy
- Python 3.10+
- colcon

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
