#!/bin/bash

# Nastaveni workspace
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"
echo "Workspace directory: $WORKSPACE_DIR"

# Sourcing ROS2

gnome-terminal \
  --tab --title="Camera Node environment" -- bash -c "
    cd $WORKSPACE_DIR
    source /opt/ros/humble/setup.bash
    source venvs/camera_pkg_venv/bin/activate
    colcon build --packages-select camera_pkg

    source install/setup.bash
    echo '=== CAMERA NODE BUILD FINISHED ==='
    exec bash
  " \
  --tab --title="RWS Node environment" -- bash -c "
    cd $WORKSPACE_DIR
    source /opt/ros/humble/setup.bash
    source venvs/abb_rws_pkg_venv/bin/activate
    colcon build --packages-select abb_rws_pkg

    source install/setup.bash
    echo '=== RWS NODE BUILD FINISHED ==='
    exec bash
  " 
#   \
#   --tab --title="Camera Node" -- bash -c "
#     cd $WORKSPACE_DIR
#     source /opt/ros/humble/setup.bash
#     source install/setup.bash
#     echo '=== CAMERA NODE ==='
#     ros2 run camera camera_node_exec
#     exec bash
#   "