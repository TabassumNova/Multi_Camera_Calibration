# Installation
- Follow (https://github.com/rameau-fr/MC-Calib)
    - Install docker (https://docs.docker.com/desktop/install/ubuntu/)
    - Install MC-Calib
```
    - docker pull bailool/mc-calib-prod
    - docker pull bailool/mc-calib-dev 
    - xhost +si:localuser:root
    - docker run \
            -ti --rm \
            --network host \
            --gpus all \
            --env="DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            --volume="$HOME/.Xauthority:/home/.Xauthority:rw" \
            --volume="${PWD}:/home/Desktop/Nova/MC_Calib" \
            --volume="PATH_TO_DATA:/home/Desktop/Nova/MC_Calib/V35" \
            bailool/mc-calib-prod
```

# Issues: 
- 
```
docker: Error response from daemon: Mounts denied: 
The path /tmp/.X11-unix is not shared from the host and is not known to Docker.
You can configure shared paths from Docker -> Preferences... -> Resources -> File Sharing.
See https://docs.docker.com/ for more info.
ERRO[0000] error waiting for container:

```
**Solution** : Docker > settings > Resources > File sharing > add /tmp

-  **New issue**: might be Nvidia problem. Check (https://www.murhabazi.com/install-nvidia-driver)
```
docker: Error response from daemon: failed to create task for container: failed to create shim task: OCI runtime create failed: runc create failed: unable to start container process: error during container init: error running hook #0: error running hook: exit status 1, stdout: , stderr: Auto-detected mode as 'legacy'
nvidia-container-cli: initialization error: load library failed: libnvidia-ml.so.1: cannot open shared object file: no such file or directory: unknown.

```
    
