

- After installing Docker container toolkit to make Docker default runtime as Nvidia, you gotta do the following (not mandatory)

```json
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
}
```

- Switching to NVIDIA graphics driver libraries with prime-select tool (reboot required). By default, it is set to on-demand in which the system selects libraries to render automatically (not always correctly). 
- So, run
`sudo prime-select nvidia` locally.


```bash
sudo systemctl restart docker
```