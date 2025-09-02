#!/bin/bash

# Nastaveni workspace
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"
echo "Workspace directory: $WORKSPACE_DIR"

# Ukončení existující session pokud existuje
tmux kill-session -t ros_build 2>/dev/null

# Vytvoření nové tmux session
tmux new-session -d -s ros_build

# Rozdělení na dva panely
tmux split-window -h

# Spuštění příkazů v jednotlivých panelech
tmux send-keys -t 0 "cd '$WORKSPACE_DIR'" Enter
tmux send-keys -t 0 "source /opt/ros/humble/setup.bash" Enter
tmux send-keys -t 0 "source venvs/camera_pkg_venv/bin/activate" Enter
tmux send-keys -t 0 "colcon build --packages-select camera_pkg" Enter
tmux send-keys -t 0 "source install/setup.bash" Enter
tmux send-keys -t 0 "echo '=== CAMERA NODE BUILD FINISHED ==='" Enter

tmux send-keys -t 1 "cd '$WORKSPACE_DIR'" Enter
tmux send-keys -t 1 "source /opt/ros/humble/setup.bash" Enter
tmux send-keys -t 1 "source venvs/abb_rws_pkg_venv/bin/activate" Enter
tmux send-keys -t 1 "colcon build --packages-select abb_rws_pkg" Enter
tmux send-keys -t 1 "source install/setup.bash" Enter
tmux send-keys -t 1 "echo '=== RWS NODE BUILD FINISHED ==='" Enter

# Připojení k session
tmux attach-session -t ros_build