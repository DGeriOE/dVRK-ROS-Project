#!/bin/bash

# Ez a sor gondoskodik róla, hogy ha nyomsz egy Ctrl+C-t, minden leálljon!
trap "kill 0" EXIT

echo "Fordítás és környezet beállítása..."
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

echo "Modulok indítása a háttérben..."
# 1. dVRK System
ros2 run dvrk_robot dvrk_system -j ~/dvrk2_ws/install/dvrk_config/share/dvrk_config/system/system-PSM1_Classic_KIN_SIMULATED.json &
sleep 2

# 2. State Publishers és RViz
ros2 launch dvrk_model arm_state_publishers.launch.py arm:=PSM1 generation:=Classic suj:=false &
ros2 run rviz2 rviz2 -d ~/dvrk2_ws/install/dvrk_model/share/dvrk_model/rviz/Classic/PSM1.rviz &
sleep 5


echo -e "
A szimuláció elindult, de ne felejtsd el \e[7m HOME \e[0m-olni a robotot, mielőtt futtatod a node-okat!" &
sleep 10

echo -e "
Ha \e[42;30m PSM1 \e[0m, akkor futtathatod a \e[1;32mmarker\e[0m és a \e[1;34mgrasp\e[0m node-ot (új terminálokban): 

\e[1;32mros2 run ros2_course interactive_marker\e[0m
ÉS
\e[1;34mros2 run ros2_course psm_interactive_grasp\e[0m" &

# Várakozás, hogy a script ne álljon le azonnal
wait
