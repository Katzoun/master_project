#!/bin/bash

# Nastaveni workspace
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

echo "Workspace directory: $WORKSPACE_DIR"

# Sourcing ROS2
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash



# Kontrola parametru use_rws (výchozí je true, takže turtle se nespustí)
# USE_RWS="true"
# if [ "$1" = "false" ]; then
#     USE_RWS="false"
#     echo "Spouštím v režimu simulace (use_rws=false) - včetně Turtle simulátoru"
# else
#     echo "Spouštím v režimu reálného robota (use_rws=true) - bez Turtle simulátoru"
# fi

# screen resolution
SCREEN_WIDTH=$(xrandr | grep '*' | awk '{print $1}' | cut -d'x' -f1 | head -1)
SCREEN_HEIGHT=$(xrandr | grep '*' | awk '{print $1}' | cut -d'x' -f2 | head -1)

# Výpočet velikosti čtvrtiny obrazovky
QUARTER_WIDTH=$((SCREEN_WIDTH / 2))
QUARTER_HEIGHT=$((SCREEN_HEIGHT / 2))

# Spuštění nodů v čtvrtinách obrazovky
# Horní levá čtvrtina - Master Node
terminator -T "Master Node" --geometry=${QUARTER_WIDTH}x${QUARTER_HEIGHT}+0+0 -e "bash -c 'cd $WORKSPACE_DIR && source /opt/ros/humble/setup.bash && source install/setup.bash && echo \"=== MASTER NODE ===\" && ros2 run master_pkg master_node_exec use_rws:=$USE_RWS; exec bash'" &

sleep 1

# Horní pravá čtvrtina - Inference Node
terminator -T "Inference Node" --geometry=${QUARTER_WIDTH}x${QUARTER_HEIGHT}+${QUARTER_WIDTH}+0 -e "bash -c 'cd $WORKSPACE_DIR && source /opt/ros/humble/setup.bash && source install/setup.bash && echo \"=== INFERENCE NODE ===\" && ros2 run camera inference_node_exec; exec bash'" &

sleep 1

# Dolní levá čtvrtina - Camera Node
terminator -T "Camera Node" --geometry=${QUARTER_WIDTH}x${QUARTER_HEIGHT}+0+${QUARTER_HEIGHT} -e "bash -c 'cd $WORKSPACE_DIR && source /opt/ros/humble/setup.bash && source install/setup.bash && echo \"=== CAMERA NODE ===\" && ros2 run camera camera_node_exec; exec bash'" &

sleep 1

# Dolní pravá čtvrtina - Keyboard Node
terminator -T "Keyboard Node" --geometry=${QUARTER_WIDTH}x${QUARTER_HEIGHT}+${QUARTER_WIDTH}+${QUARTER_HEIGHT} -e "bash -c 'cd $WORKSPACE_DIR && source /opt/ros/humble/setup.bash && source install/setup.bash && echo \"=== KEYBOARD NODE ===\" && ros2 run master_pkg keyboard_publisher_exec; exec bash'" &

sleep 1