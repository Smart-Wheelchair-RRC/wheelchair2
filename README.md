# Auto-Boot for Ducky

This setup ensures that the **ROS2 nodes** inside Docker start automatically when the Jetson boots.

---

Before setting up auto-boot, build the Docker image from the `wheelchair` folder:

```bash
cd ~/wheelchair
docker build -t wheelchair-boot .
```

## 1.Docker Entrypoint
The `entrypoint.sh` script has been added to the **Dockerfile** so that the `docker run` command will automatically launch all ROS2 nodes.

---

## 2.System-Level Auto-Start

We create a **startup script** that runs the `docker run` command at boot.

### Create the startup script
```bash
cd /usr/local/bin
sudo nano start_mvp.sh
```
The [`start_mvp.sh`](dependencies/auto-boot/start_mvp.sh) script is available in the [`dependencies/auto-boot`](dependencies/auto-boot) folder.


Make the script executable:
```bash
sudo chmod +x /usr/local/bin/start_mvp.sh
```

### Create a Systemd Service
```bash
cd /etc/systemd/system
sudo nano mvp.service
```
The mvp.service file is also available in the main wheelchair folder.

### Enable & Start the Service
```bash
# Reload systemd to recognize new service
sudo systemctl daemon-reload

# Enable the service to start at boot
sudo systemctl enable mvp.service

# Start the service now
sudo systemctl start mvp.service

# Check service status
sudo systemctl status mvp.service
```

---

## 🛠 Troubleshooting

### Ethernet Taking Priority Over Wi-Fi (ROS2 DDS Network Issue)

By default, Linux may give **Ethernet** a higher routing priority than **Wi-Fi**.  
If your LiDAR is connected via Ethernet, the DDS middleware might choose the Ethernet interface over Wi-Fi for ROS communication — even if you want Wi-Fi to be used for peer-to-peer ROS2 traffic.

#### Symptoms:
- `ros2 topic list` shows no data over Wi-Fi on another ROS2 machine with same Domain ID.
- `ip route show` shows Ethernet with a lower metric (higher priority) than Wi-Fi.
- DDS discovery only works when Ethernet is unplugged.

#### Solution: Increase Ethernet Metric or Lower Wi-Fi Metric
You can change network priority by adjusting connection metrics.

1. Check current metrics:
```bash
ip route show
```
2.Identify your connection names:
```bash
nmcli connection show
```
3.Increase the metric for Ethernet (so Wi-Fi is preferred):
```bash
nmcli connection modify "<Ethernet_Connection_Name>" ipv4.route-metric 200
```
4.Lower the metric for Wi-Fi:
```bash
nmcli connection modify "<WiFi_Connection_Name>" ipv4.route-metric 100
```
5.Restart NetworkManager:
```bash
sudo systemctl restart NetworkManager
```
Verification:
```bash
ip route show
```












