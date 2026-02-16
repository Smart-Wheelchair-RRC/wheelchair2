#!/bin/bash

# log commands and their arguments as they are executed
# exit on first failure
set -ex

# set up bash shortcuts by sourcing the shortcuts script in bashrc
echo "source /home/container_user/wheelchair2/src/scripts/shortcuts.sh" >> /home/container_user/.bashrc