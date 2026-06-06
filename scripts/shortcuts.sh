#!/bin/bash

alias wss="source /home/container_user/wheelchair2/install/setup.bash && echo \"Sourced wheelchair2 workspace.\" && cd /home/container_user/wheelchair2/src"
alias wsb="cd /home/container_user/wheelchair2 && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && wss"
alias wsd="rm -rf /home/container_user/wheelchair2/build /home/container_user/wheelchair2/install /home/container_user/wheelchair2/log && echo \"Deleted build, install, and log directories.\""