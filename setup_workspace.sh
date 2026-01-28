#!/bin/bash
# Workspace setup script - sources ROS2 and workspace, overlays venv packages

source /opt/ros/jazzy/setup.bash

# Add venv site-packages to PYTHONPATH so system Python can find venv packages
if [ -d ".venv/lib" ]; then
    VENV_SITE_PACKAGES=$(find .venv/lib -type d -name "site-packages" 2>/dev/null | head -1)
    if [ -n "$VENV_SITE_PACKAGES" ]; then
        export PYTHONPATH="$VENV_SITE_PACKAGES:$PYTHONPATH"
    fi
fi

if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

echo "Workspace environment activated!"
echo "  ROS2: Jazzy"
echo "  Python packages: $([ -n "$VENV_SITE_PACKAGES" ] && echo "venv overlay + system" || echo "system only")"
echo "  Workspace: $([ -f install/setup.bash ] && echo 'sourced' || echo 'not built yet')"
