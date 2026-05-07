#!/bin/bash

# export MAKEFLAGS="-j1" && echo "Set MAKEFLAGS to -j1 for sequential builds."
alias wss="source /home/container_user/wheelchair2/install/setup.bash  && echo \"Sourced wheelchair2 workspace.\""
alias wsb="cd /home/container_user/wheelchair2 && export MAKEFLAGS=-j4 && colcon build --parallel-workers 2 --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && wss"
alias wsd="rm -rf /home/container_user/wheelchair2/build /home/container_user/wheelchair2/install /home/container_user/wheelchair2/log && echo \"Deleted build, install, and log directories.\""