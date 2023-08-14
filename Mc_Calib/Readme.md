# Installation
- Follow (https://github.com/rameau-fr/MC-Calib)
    - Install docker (https://docs.docker.com/desktop/install/ubuntu/)

    '''
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
            
    '''
    https://www.murhabazi.com/install-nvidia-driver
