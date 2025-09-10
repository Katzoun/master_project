#!/bin/bash
# filepath: /home/tomas/master_project/scripts/build.sh

# workspace setup
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"
echo "Workspace directory: $WORKSPACE_DIR"
echo "Starting build process..."
SESSION_NAME="ros_build"

# kill existing session
tmux kill-session -t $SESSION_NAME 2>/dev/null

# new tmux session
tmux new-session -d -s $SESSION_NAME -n "general"


# general workspace window
tmux select-window -t $SESSION_NAME:general
tmux send-keys -t $SESSION_NAME:general "cd '$WORKSPACE_DIR'" Enter
tmux send-keys -t $SESSION_NAME:general "source /opt/ros/humble/setup.bash" Enter
tmux send-keys -t $SESSION_NAME:general "echo '=== CURRENT PYTHON PATH ==='" Enter
tmux send-keys -t $SESSION_NAME:general "which python" Enter
tmux send-keys -t $SESSION_NAME:general "echo '=== BUILDING INTERFACE PKG ==='" Enter
tmux send-keys -t $SESSION_NAME:general "colcon build --packages-select interface_pkg" Enter
tmux send-keys -t $SESSION_NAME:general "echo '=== INTERFACE BUILD FINISHED ==='" Enter
# tmux send-keys -t $SESSION_NAME:general "source install/setup.bash" Enter
tmux send-keys -t $SESSION_NAME:general "echo '=== GENERAL WORKSPACE READY ==='" Enter
# tmux send-keys -t $SESSION_NAME:general "echo 'Available commands: ros2 run, ros2 launch, etc.'" Enter

# Wait until the commands in the "general" window finish before proceeding
tmux wait-for -S general_build_done
tmux send-keys -t $SESSION_NAME:general "tmux wait-for general_build_done" Enter


tmux select-window -t $SESSION_NAME:camera_build
tmux new-window -t $SESSION_NAME -n "camera_build"
tmux send-keys -t $SESSION_NAME:camera_build "cd '$WORKSPACE_DIR'" Enter
tmux send-keys -t $SESSION_NAME:camera_build "source /opt/ros/humble/setup.bash" Enter
tmux send-keys -t $SESSION_NAME:camera_build "source venvs/camera_pkg_venv/bin/activate" Enter
tmux send-keys -t $SESSION_NAME:camera_build "echo '=== CURRENT PYTHON PATH ==='" Enter
tmux send-keys -t $SESSION_NAME:camera_build "which python" Enter
tmux send-keys -t $SESSION_NAME:camera_build "echo '=== BUILDING CAMERA PKG ==='" Enter
tmux send-keys -t $SESSION_NAME:camera_build "colcon build --packages-select camera_pkg" Enter
tmux send-keys -t $SESSION_NAME:camera_build "source install/setup.bash" Enter
tmux send-keys -t $SESSION_NAME:camera_build "echo '=== CAMERA NODE BUILD FINISHED ==='" Enter
tmux send-keys -t $SESSION_NAME:camera_build "echo 'Environment ready for camera_pkg development'" Enter

tmux wait-for -S camera_build_done
tmux send-keys -t $SESSION_NAME:camera_build "tmux wait-for camera_build_done" Enter

# new window for abb_rws_pkg
tmux select-window -t $SESSION_NAME:abb_rws_build
tmux new-window -t $SESSION_NAME -n "abb_rws_build"
tmux send-keys -t $SESSION_NAME:abb_rws_build "cd '$WORKSPACE_DIR'" Enter
tmux send-keys -t $SESSION_NAME:abb_rws_build "source /opt/ros/humble/setup.bash" Enter
tmux send-keys -t $SESSION_NAME:abb_rws_build "source venvs/abb_rws_pkg_venv/bin/activate" Enter
tmux send-keys -t $SESSION_NAME:abb_rws_build "echo '=== CURRENT PYTHON PATH ==='" Enter
tmux send-keys -t $SESSION_NAME:abb_rws_build "which python" Enter
tmux send-keys -t $SESSION_NAME:abb_rws_build "echo '=== BUILDING ABB RWS PKG ==='" Enter
tmux send-keys -t $SESSION_NAME:abb_rws_build "colcon build --packages-select abb_rws_pkg" Enter
tmux send-keys -t $SESSION_NAME:abb_rws_build "source install/setup.bash" Enter
tmux send-keys -t $SESSION_NAME:abb_rws_build "echo '=== RWS NODE BUILD FINISHED ==='" Enter
tmux send-keys -t $SESSION_NAME:abb_rws_build "echo 'Environment ready for abb_rws_pkg development'" Enter

tmux send-keys -t $SESSION_NAME:general "source install/setup.bash" Enter
tmux send-keys -t $SESSION_NAME:camera_build "ros2 run camera_pkg photoneo_node_exec" Enter

tmux wait-for -S abb_rws_build_done
tmux send-keys -t $SESSION_NAME:abb_rws_build "tmux wait-for abb_rws_build_done" Enter



echo "=== Build started in tmux session '$SESSION_NAME' ==="
echo "Use these commands to interact with the session:"
echo "  tmux attach -t $SESSION_NAME      # Attach to session"
echo "  tmux list-windows -t $SESSION_NAME # List windows"
echo "  Ctrl+b + c                        # Create new window"
echo "  Ctrl+b + n                        # Next window"
echo "  Ctrl+b + p                        # Previous window"
echo "  Ctrl+b + d                        # Detach from session"

# Attach to the session
tmux attach-session -t $SESSION_NAME