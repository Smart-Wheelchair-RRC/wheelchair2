# Deployment Docker Setup for NVIDIA Jetson

This directory contains utility scripts to build and run the Wheelchair2 navigation container on an NVIDIA Jetson platform **without VS Code**.

## Files

| File | Purpose |
|---|---|
| `build.sh` | Builds the Docker image from `.devcontainer/jetson-arm/Dockerfile`, compiles the entire ROS 2 workspace inside the image, and tags it as `wheelchair2:jetson`. |
| `run.sh` | Runs the container with name `wheelchair_nav` using the built image. Configures GPU access, device pass-through, host networking, and bind-mounts the workspace source. |
| `docker-compose.yml` | A Docker Compose alternative to build and run the container. |

---

## Prerequisites

### 1. Hardware Checklist
Before starting the container, ensure:
- The wheelchair main power switch is **ON**.
- The **Livox MID-360 LiDAR** is powered and its ethernet cable is connected to the Jetson.
- The **USB serial cable** (Kangaroo motor controller, typically `/dev/ttyACM0`) is plugged into the Jetson.

### 2. OS-Level Socket Buffer Tuning (One-Time Setup)

CycloneDDS requires large UDP socket buffers (10 MB) to handle 3D PointCloud data without packet loss. The Linux default (`208 KB`) is far too small and will cause **all ROS 2 nodes to crash on startup**.

Run the following on **both the Jetson host and the remote host**:

```bash
# Apply immediately
sudo sysctl -w net.core.rmem_max=10485760
sudo sysctl -w net.core.wmem_max=10485760

# Make persistent across reboots
echo "net.core.rmem_max=10485760" | sudo tee -a /etc/sysctl.conf
echo "net.core.wmem_max=10485760" | sudo tee -a /etc/sysctl.conf
```

> **TIP?** To check what your operating system's baseline limit is currently, run (`cat /proc/sys/net/core/rmem_max`)

> **Why?** CycloneDDS requests 10 MB socket buffers at initialization. If the OS kernel cap (`rmem_max`) is lower than the requested size, CycloneDDS throws a hard error and refuses to create any DDS participant — killing every ROS 2 node before it starts.

---

## Quick Start (Jetson)

### Step 1: Build the Docker Image

From the repository root on the Jetson:

```bash
cd ~/wheelchair_ws/wheelchair2
./docker/build.sh
```

This compiles the full ROS 2 workspace inside the image (takes ~7 minutes on Jetson Orin). You only need to rebuild when source code or dependencies change.

### Step 2: Start the Container

```bash
./docker/run.sh -r tail -f /dev/null
```

| Flag | Effect |
|---|---|
| `-r` / `--restart` | Force-removes any existing `wheelchair_nav` container and starts fresh. |
| `--no-mount` | Run from the code baked into the image instead of bind-mounting the host workspace. |
| (no args) | Opens an interactive bash shell. If the container is already running, attaches a new shell. |

The workspace source is bind-mounted from the host (`~/wheelchair_ws/wheelchair2`) into the container at `/home/container_user/wheelchair2/src`, so any file changes on the host (e.g., `git pull`) are instantly visible inside the container.

### Step 3: Launch the Navigation Stack

Attach to the container and run the tmux launcher:

```bash
docker exec -it -u container_user wheelchair_nav bash
cd ~/wheelchair2/src/wheelchair2_navigation/shell_scripts/
./nav_3D_loc.tmux
```

This creates a `tmux` session called `wheelchair_navigation` with two windows:

| Window | Pane | What it runs |
|---|---|---|
| `bringup` (0) | Left | Livox LiDAR driver + ground detection + pointcloud-to-laserscan |
| `bringup` (0) | Right | Kangaroo motor controller + robot state publisher + joystick teleop |
| `loc_nav` (1) | Left | DLIO odometry + ICP localization + 3D-BBS global localizer |
| `loc_nav` (1) | Right | Nav2 stack (planner, controller, costmaps, behavior trees) |

To **re-attach** to the tmux session later:

```bash
docker exec -it -u container_user wheelchair_nav bash
tmux attach -t wheelchair_navigation
```

> **Tip:** Use `Ctrl+B` then `n` to switch between tmux windows, and `Ctrl+B` then arrow keys to switch between panes.

---

## Running RViz on the Laptop

### Prerequisites

1. Ensure the laptop is connected to the Jetson's Wi-Fi hotspot (`10.42.0.x` subnet).
2. Ensure the OS socket buffer limits are set (see [Socket Buffer Tuning](#2-os-level-socket-buffer-tuning-one-time-setup) above).
3. The `wheelchair2` repository is cloned on the laptop at `~/wheelchair_ws/wheelchair2`.

### Launch RViz

From the laptop terminal:

```bash
cd ~/wheelchair_ws/wheelchair2
./wheelchair2_navigation/shell_scripts/rviz_nav_unsecure.sh
```

This script:
1. Builds `wheelchair2_navigation` and `wheelchair_intelligence_ros2` packages locally.
2. Sources the ROS 2 environment.
3. Sets CycloneDDS as the RMW, loads `cyclonedds_laptop.xml`, and disables SROS2.
4. Launches RViz with the pre-configured `navigation.rviz` display file.

### Using RViz

1. **Set Initial Pose**: Click the **2D Pose Estimate** button in the toolbar. Click and drag on the map at the robot's approximate physical location and heading. This initializes the ICP localization.
2. **Send a Navigation Goal**: Click the **2D Goal Pose** button. Click and drag on the map where you want the wheelchair to navigate.
3. **Monitor**: The laser scan (`/scan`), costmaps, and planned path should appear. The wheelchair will begin moving autonomously.

---

## CycloneDDS Network Configuration

Communication between the Jetson and the laptop uses **Eclipse Cyclone DDS** in **unicast mode** (multicast is disabled to prevent Wi-Fi saturation). The configuration files are:

| File | Used by | Network Interface |
|---|---|---|
| `scripts/cyclonedds_jetson.xml` | Jetson container | `wlxb01921369c85` (Wi-Fi hotspot AP) |
| `scripts/cyclonedds_laptop.xml` | Laptop | `wlp3s0` (Wi-Fi client) |

Key settings in both files:

```xml
<AllowMulticast>false</AllowMulticast>           <!-- Prevent Wi-Fi saturation -->
<SocketReceiveBufferSize min="10MB" />            <!-- Handle large PointClouds -->
<MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>  <!-- Support many nodes -->
<Peers>
  <Peer address="10.42.0.1" />   <!-- Jetson hotspot IP -->
  <Peer address="10.42.0.85" />  <!-- Laptop IP on hotspot -->
</Peers>
```

### Changing Wi-Fi Interface Names or IPs

If the wireless adapter changes (e.g., different USB dongle, different laptop), you need to update the CycloneDDS XML files with the new interface name and IP address.

#### How to Find Your Interface Name and IP

Run `ip addr` on the device. Example output from the laptop:

```
1: lo: <LOOPBACK,UP,LOWER_UP> ...           ← Ignore (loopback, internal only)
    inet 127.0.0.1/8 ...

2: eno1: <NO-CARRIER,...> state DOWN ...     ← Ignore (ethernet, cable unplugged)
    link/ether 60:cf:84:b8:71:ec ...

3: wlp3s0: <BROADCAST,...> state UP ...      ← ✅ This is the Wi-Fi interface
    inet 10.42.0.85/24 ...                   ← ✅ This is the IP to use

4: docker0: <NO-CARRIER,...> state DOWN ...  ← Ignore (Docker bridge, internal)
    inet 172.17.0.1/16 ...
```

**Rules to identify the correct entry:**
1. Look for an interface with **state `UP`** (not `DOWN`).
2. Ignore `lo` (loopback), `docker0` / `br-*` (Docker bridges), and any `veth*` (virtual interfaces).
3. The Wi-Fi interface is typically named `wlp*`, `wlan*`, or `wlx*` (USB dongles).
4. The **`inet` line** under that interface shows the IP address (e.g., `10.42.0.85`).
5. The **interface name** itself (e.g., `wlp3s0`) goes into `<NetworkInterface name="..." />`.

#### Where to Update

| What to change | File | XML tag |
|---|---|---|
| Jetson Wi-Fi interface name | `scripts/cyclonedds_jetson.xml` | `<NetworkInterface name="wlxb01921369c85" />` |
| Laptop Wi-Fi interface name | `scripts/cyclonedds_laptop.xml` | `<NetworkInterface name="wlp3s0" />` |
| Jetson IP address | Both XML files | `<Peer address="10.42.0.1" />` |
| Laptop IP address | Both XML files | `<Peer address="10.42.0.85" />` |

> **Important:** The `<Peers>` section must list **both** IP addresses (Jetson and laptop) in **both** XML files. This is because CycloneDDS needs to discover nodes running on the local machine as well as the remote peer.

---

## Docker Compose Alternative

If you prefer standard Docker Compose commands:

```bash
# Start the container in the background
docker compose -f docker/docker-compose.yml up -d

# Open an interactive shell
docker exec -it -u container_user wheelchair_nav bash

# Stop the container
docker compose -f docker/docker-compose.yml down
```

---

## Troubleshooting

### All ROS 2 nodes crash with `failed to increase socket receive buffer size`

The OS socket buffer limit is too low. Run:
```bash
sudo sysctl -w net.core.rmem_max=10485760
sudo sysctl -w net.core.wmem_max=10485760
```

### `Failed to find a free participant index for domain 56`

Too many ROS 2 nodes are running for the default participant limit. Ensure `<MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>` is present in the CycloneDDS XML file.

### `ros2 topic list` only shows `/rosout` and `/parameter_events`

- Verify `CYCLONEDDS_URI` is exported and points to the correct XML file.
- Verify `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is exported.
- Verify `ROS_DOMAIN_ID=56` matches on both machines.
- Verify both machines are on the same Wi-Fi subnet (`10.42.0.x`).

### Wi-Fi drops when the LiDAR starts streaming

Multicast is enabled (the default). Ensure `<AllowMulticast>false</AllowMulticast>` is set in both CycloneDDS XML files.

### All processes show `<defunct>` inside the container

A GUI node (like `rviz2`) was started inside the headless container and crashed, killing the parent shell and turning all backgrounded nodes into zombies. Ensure `nav2_rviz_required:=False` and `3dbbs_rviz:=false` are set in the launch files (this is already configured in `nav_3D_loc.tmux`).

### LiDAR driver starts but publishes no data

Check the LiDAR IP in `dependencies/livox_ros_driver2/config/MID360_config.json`. The `"lidar_configs"` → `"ip"` field must match the physical LiDAR's IP address (currently `192.168.1.192`).
