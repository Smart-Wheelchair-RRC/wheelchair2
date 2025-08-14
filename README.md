# Auto-Boot for Ducky

This setup ensures that the **ROS2 nodes** inside Docker start automatically when the Jetson boots.

---

Before setting up auto-boot, build the [`Docker image`](https://github.com/Smart-Wheelchair-RRC/wheelchair2/blob/auto-boot/Dockerfile) from the **Dockerfile**:

```bash
cd ~/wheelchair
docker build -t wheelchair-boot .
```

## 1.Docker Entrypoint
The [`entrypoint.sh`](https://github.com/Smart-Wheelchair-RRC/wheelchair2/blob/auto-boot/dependencies/auto_boot/entrypoint.sh) script has been added to the **Dockerfile** so that the `docker run` command will automatically launch all ROS2 nodes.

---

## 2.System-Level Auto-Start

We create a **startup script**([`start_mvp.sh`](https://github.com/Smart-Wheelchair-RRC/wheelchair2/blob/auto-boot/dependencies/auto_boot/start_mvp.sh)) that runs the `docker run` command at boot.

### Create the startup script
```bash
cd /usr/local/bin
sudo nano start_mvp.sh
```
Make the script executable:
```bash
sudo chmod +x /usr/local/bin/start_mvp.sh
```

### Create a Systemd Service
we then create a [`mvp.service`](https://github.com/Smart-Wheelchair-RRC/wheelchair2/blob/auto-boot/dependencies/auto_boot/mvp.service) that runs the startup script on boot.
```bash
cd /etc/systemd/system
sudo nano mvp.service
```
Edit the [`user`](https://github.com/Smart-Wheelchair-RRC/wheelchair2/blob/713a9fa01c623aa3fce9413c724efe130cfeebf1/dependencies/auto_boot/mvp.service#L11)accordingly

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

### 1.Ethernet Taking Priority Over Wi-Fi (ROS2 DDS Network Issue)

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
### 2.ROS2 control manager fails
for unknown reasons sometimes the ros2 control manager fails to start up because it doesnt have the required permissions to access Kangaroo x2 motion controller.
In that case from your remote machine ssh into ducky(if your local machine and ducky are connected to wheelchair@rrxc ,then follow the instructions below):
```bash
ssh ducky@192.168.0.178
```
open a terminal(CTRL+ALT+t)
attach to the running docker container like so : 
```bash
docker exec -it mvp_test bash
```
once inside the container attach to the running tmux window `navigation_real`
```bash
tmux attach -t navigation_real
```















