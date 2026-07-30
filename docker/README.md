# Standalone Docker Setup for NVIDIA Jetson

This directory contains utility scripts to build and run the Wheelchair2 development container on an NVIDIA Jetson platform without using VS Code.

## Files

1. **`build.sh`**: Builds the Docker image from `.devcontainer/jetson-arm/Dockerfile` and tags it as `wheelchair2:jetson`.
2. **`run.sh`**: Runs the container with name `wheelchair_nav` using the built image and configures GUI display forwarding, GPU access, system parameters, and network settings matching `devcontainer.json`.
3. **`docker-compose.yml`**: A Docker Compose alternative to build and run the container.

---

## How to Use

### 1. Build the Docker Image
To build the Docker image with your project code embedded inside it, run the build script from the repository root:
```bash
./docker/build.sh
```

### 2. Start and Enter the Container
To start the container (or attach to it if it is already running) and open an interactive bash shell, run:
```bash
./docker/run.sh
```

* **Attaching Multiple Shells**: If the container `wheelchair_nav` is already running, running `./docker/run.sh` again will automatically open a new interactive terminal shell inside the existing running container. This allows you to work across multiple terminals without restarting the container or stopping active ROS nodes.
* **Force Restarting**: To stop and delete the running container and start a fresh one, run with the `-r` or `--restart` option:
  ```bash
  ./docker/run.sh -r
  ```
* **No Mount Mode**: By default, the workspace is bind-mounted from the host (`$(pwd)`) to `/home/container_user/wheelchair2/src` so that changes made on the host are instantly reflected inside the container. To run the container standalone using the files embedded inside the image, use the `--no-mount` option:
  ```bash
  ./docker/run.sh --no-mount
  ```
* **Running Commands Directly**: You can pass commands to run them inside the container directly:
  ```bash
  ./docker/run.sh ros2 launch wheelchair2_bringup bringup.launch.py
  ```

---

### 3. Alternative: Using Docker Compose
If you prefer standard Docker Compose commands:

* **Start the container in the background**:
  ```bash
  docker compose -f docker/docker-compose.yml up -d
  ```
* **Open an interactive shell**:
  ```bash
  docker exec -it -u container_user wheelchair_nav bash
  ```
* **Stop the container**:
  ```bash
  docker compose -f docker/docker-compose.yml down
  ```
