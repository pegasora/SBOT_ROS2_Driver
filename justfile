# StandardBot ROS2 Driver - Justfile
# Common commands for building and running the ROS2 workspace

# Show available commands
default:
    @just --list

# Setup venv and install dependencies (run once)
setup:
    @echo "Setting up Python virtual environment..."
    @if [ -d ".venv" ]; then \
        echo "✓ .venv already exists"; \
    else \
        python3 -m venv .venv && echo "✓ Created .venv"; \
    fi
    @echo "Installing Python dependencies..."
    @bash -c "source .venv/bin/activate && pip install -e . && deactivate"
    @echo ""
    @echo "✓ Setup complete!"
    @echo ""
    @echo "Next steps:"
    @echo "  1. source setup_workspace.sh"
    @echo "  2. just build"
    @echo "  3. just launch"

# Clean build artifacts
clean:
    @echo "Cleaning build artifacts..."
    rm -rf build install log
    @echo "✓ Cleaned build/, install/, log/"

# Build the ROS2 workspace
build:
    @echo "Building ROS2 workspace..."
    colcon build --symlink-install
    @echo ""
    @echo "✓ Build complete!"
    @echo "To use the workspace, run:" 
    @echo " or you can manually run: source install/setup.bash"
    

# Clean and rebuild everything
rebuild: clean build

# Launch the StandardBot demo
launch *ARGS:
    @echo "Launching StandardBot drivers..."
    @bash -c "source install/setup.bash && ros2 launch standard_bot_bringup demo.launch.py {{ARGS}}"

# Source the workspace (prints command to eval)
source:
    @echo "Run this command to source the workspace:"
    @echo "  source install/setup.bash"

# Check ROS2 environment
check-env:
    @echo "ROS2 Environment Check:"
    @echo "======================="
    @bash -c 'echo "ROS_DISTRO: ${ROS_DISTRO:-not set}"'
    @bash -c 'echo "ROS_VERSION: ${ROS_VERSION:-not set}"'
    @bash -c 'echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-not set}"'
    @which ros2 > /dev/null && echo "✓ ros2 command found" || echo "✗ ros2 command not found"
    @which colcon > /dev/null && echo "✓ colcon command found" || echo "✗ colcon command not found"

# Install Python dependencies from pyproject.toml (using pip)
install-deps:
    @echo "Installing Python dependencies with pip..."
    pip install -e .

# Install Python dependencies using uv (faster alternative)
install-deps-uv:
    @echo "Installing Python dependencies with uv..."
    @which uv > /dev/null || (echo "✗ uv not found. Install with: curl -LsSf https://astral.sh/uv/install.sh | sh" && exit 1)
    uv pip install -e .

# Show workspace information
info:
    @echo "StandardBot ROS2 Driver Workspace"
    @echo "=================================="
    @echo "Packages:"
    @echo "  - action_interfaces    (Custom action definitions)"
    @echo "  - sbot_interfaces      (Custom messages/services)"
    @echo "  - sb_controller        (Robot controller nodes)"
    @echo "  - standard_bot_bringup (Launch files)"
    @echo ""
    @echo "Quick Start:"
    @echo "  1. just setup"
    @echo "  2. just env      (or: source setup_workspace.sh)"
    @echo "  3. just build"
    @echo "  4. just launch"
