#!/bin/bash

# Nastaveni workspace
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"
echo "Workspace directory: $WORKSPACE_DIR"

# Vytvoření více oken místo tabů
gnome-terminal --window -- bash -c "
    cd '$WORKSPACE_DIR'
    source /opt/ros/humble/setup.bash
    source venvs/camera_pkg_venv/bin/activate
    echo '=== CURRENT PYTHON PATH ==='
    which python
    colcon build --packages-select camera_pkg
    source install/setup.bash
    echo '=== CAMERA NODE BUILD FINISHED ==='
    exec bash
" &

gnome-terminal --window -- bash -c "
    cd '$WORKSPACE_DIR'
    
    source /opt/ros/humble/setup.bash
    source venvs/abb_rws_pkg_venv/bin/activate
    echo '=== CURRENT PYTHON PATH ==='
    which python

    colcon build --packages-select abb_rws_pkg
    source install/setup.bash
    echo '=== RWS NODE BUILD FINISHED ==='
    exec bash
" &

wait