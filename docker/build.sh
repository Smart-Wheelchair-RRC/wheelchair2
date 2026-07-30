#!/bin/bash
# Exit on error
set -e

# Get the directory of the script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"

cd "$WORKSPACE_DIR"

echo "Building Wheelchair2 Docker image for Jetson-arm..."
docker build -t wheelchair2:jetson -f .devcontainer/jetson-arm/Dockerfile .
echo "Build complete! Tagged as wheelchair2:jetson"
