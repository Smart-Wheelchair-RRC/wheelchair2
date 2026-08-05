#!/bin/bash
# Exit on error
set -e

# Get the directory of the script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"

CONTAINER_NAME="wheelchair_nav"
IMAGE_NAME="wheelchair2:jetson"

# Print usage instructions
usage() {
    echo "Usage: $0 [options] [command]"
    echo "Options:"
    echo "  -h, --help      Show this help message"
    echo "  -r, --restart   Force stop and remove existing container before running"
    echo "  --no-mount      Do not bind-mount the workspace (run from code copied inside image)"
    echo "  -s, --shell     Open a bash shell inside the container (default)"
    echo ""
    echo "Examples:"
    echo "  $0              Starts/attaches to container and opens interactive bash"
    echo "  $0 -r           Restarts container and opens interactive bash"
    echo "  $0 ros2 launch wheelchair2_bringup bringup.launch.py"
}

# Parse command line options
RESTART=false
MOUNT_WORKSPACE=true
COMMAND=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        -r|--restart)
            RESTART=true
            shift
            ;;
        --no-mount)
            MOUNT_WORKSPACE=false
            shift
            ;;
        -s|--shell)
            COMMAND="bash"
            shift
            ;;
        *)
            # The remaining arguments are the command to run
            COMMAND="$@"
            break
            ;;
    esac
done

if [ -z "$COMMAND" ]; then
    COMMAND="bash"
fi

# Stop and remove existing container if restart is requested
if [ "$RESTART" = true ]; then
    if docker ps -a --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}$"; then
        echo "Stopping and removing existing container '${CONTAINER_NAME}'..."
        docker stop "${CONTAINER_NAME}" >/dev/null 2>&1 || true
        docker rm "${CONTAINER_NAME}" >/dev/null 2>&1 || true
    fi
fi

# Check if the container is already running
IS_RUNNING=false
if docker ps --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}$"; then
    IS_RUNNING=true
fi

# Check if the container exists but is stopped
EXISTS_BUT_STOPPED=false
if ! $IS_RUNNING; then
    if docker ps -a --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}$"; then
        EXISTS_BUT_STOPPED=true
    fi
fi

# Function to run post-start setup
run_post_start_setup() {
    echo "Running post-start permissions setup..."
    docker exec -u root "${CONTAINER_NAME}" bash -c "mkdir -p /home/container_user/.ros && chown -R container_user:container_user /home/container_user/wheelchair2 /run /home/container_user/.ros"
    
    echo "Running post-create configuration script..."
    docker exec -u container_user "${CONTAINER_NAME}" bash /home/container_user/wheelchair2/src/scripts/post_create.sh
}

if $IS_RUNNING; then
    echo "Container '${CONTAINER_NAME}' is already running. Executing command..."
else
    if $EXISTS_BUT_STOPPED; then
        echo "Starting existing stopped container '${CONTAINER_NAME}'..."
        docker start "${CONTAINER_NAME}"
        run_post_start_setup
    else
        echo "Creating and starting new container '${CONTAINER_NAME}'..."
        
        # Enable GUI/RViz access on the host
        xhost +local:root >/dev/null 2>&1 || echo "Warning: Could not configure xhost. GUI apps might not work."
        
        # Setup environment variables matching devcontainer.json
        ENV_OPTS=(
            -e DISPLAY="${DISPLAY:-:0}"
            -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY}"
            -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}"
            -e PULSE_SERVER="${PULSE_SERVER}"
            -e QT_X11_NO_MITSHM=1
            -e ROS_DOMAIN_ID=56
            -e NVIDIA_VISIBLE_DEVICES=all
            -e NVIDIA_DRIVER_CAPABILITIES=all
            -e NVIDIA_DISABLE_REQUIRE=true
        )
        
        # Setup run arguments matching devcontainer.json
        RUN_ARGS=(
            --runtime=nvidia
            --privileged
            --cap-add=SYS_NICE
            --security-opt=seccomp:unconfined
            --security-opt=apparmor:unconfined
            -v /dev:/dev
            -v /tmp/.X11-unix:/tmp/.X11-unix
            -v "/home/ducky/.ros/sros2:/home/container_user/.ros/sros2"
            --net=host
            --ipc=host
            -v /dev/pts:/dev/pts
            --user container_user
        )
        
        # Configure mount options
        MOUNT_ARGS=()
        if [ "$MOUNT_WORKSPACE" = true ]; then
            echo "Mounting workspace: $WORKSPACE_DIR -> /home/container_user/wheelchair2/src"
            MOUNT_ARGS+=(-v "$WORKSPACE_DIR:/home/container_user/wheelchair2/src")
        else
            echo "Running without mounting workspace (using files copied in the image)."
        fi
        
        # Verify if the target image exists, if not build it or use the base image
        if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}$"; then
            echo "Image '${IMAGE_NAME}' not found."
            echo "Checking if we can pull the base image or build..."
            # Check if Dockerfile exists
            if [ -f "$WORKSPACE_DIR/.devcontainer/jetson-arm/Dockerfile" ]; then
                echo "Dockerfile found. Building the image..."
                "$SCRIPT_DIR/build.sh"
            else
                echo "Dockerfile not found. Falling back to the base image from GitHub container registry."
                IMAGE_NAME="ghcr.io/smart-wheelchair-rrc/wheelchair2_base_jetson:dev5.2.1"
            fi
        fi
        
        # Start container in detached mode to allow executing setup scripts
        docker run -d \
            --name "${CONTAINER_NAME}" \
            --entrypoint "" \
            "${RUN_ARGS[@]}" \
            "${ENV_OPTS[@]}" \
            "${MOUNT_ARGS[@]}" \
            "${IMAGE_NAME}" \
            tail -f /dev/null
            
        # Run the post start setup
        run_post_start_setup
    fi
fi

# Execute command inside container
if [ "$COMMAND" = "bash" ]; then
    echo "Entering interactive shell in container '${CONTAINER_NAME}'..."
    docker exec -it -u container_user "${CONTAINER_NAME}" bash
else
    echo "Executing command in container '${CONTAINER_NAME}': $COMMAND"
    docker exec -it -u container_user "${CONTAINER_NAME}" $COMMAND
fi
