# Source from ~/.bashrc on the Jetson; paths match the deployed ~/ros2_ws layout.
source /opt/ros/jazzy/setup.bash

# Bridge may be built separately inside the communication repository.
if [ -f "$HOME/ros2_ws/src/Jetson_ArmPi_Communication/ros2_ws/install/local_setup.bash" ]; then
    source "$HOME/ros2_ws/src/Jetson_ArmPi_Communication/ros2_ws/install/local_setup.bash"
fi

# Main workspace: bringup, vision, SLAM description and any built VLA packages.
if [ -f "$HOME/ros2_ws/install/local_setup.bash" ]; then
    source "$HOME/ros2_ws/install/local_setup.bash"
fi

export ARMPI_PROTOCOL_PATH="$HOME/ros2_ws/src/Jetson_ArmPi_Communication/protocol"
