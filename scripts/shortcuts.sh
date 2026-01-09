#!/bin/bash

alias wss="source /home/container_user/wheelchair2/install/setup.bash && echo \"Sourced wheelchair2 workspace.\""
alias wsb="cd /home/container_user/wheelchair2 && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && wss"
