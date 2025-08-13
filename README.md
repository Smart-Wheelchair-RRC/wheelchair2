# Auto-Boot for Ducky

This setup ensures that the **ROS2 nodes** inside Docker start automatically when the Jetson boots.

---

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
The start_mvp.sh script is available in the main wheelchair folder.

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
