# Installation
- Follow (https://github.com/rameau-fr/MC-Calib)
    - Install docker (https://docs.docker.com/desktop/install/ubuntu/)
    '''
    docker pull bailool/mc-calib-prod
    docker pull bailool/mc-calib-dev 
    xhost +si:localuser:root
    docker run \
            --runtime=nvidia \
            -ti --rm \
            --network host \
            --gpus all \
            --env="DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            --volume="$HOME/.Xauthority:/home/.Xauthority:rw" \
            --volume="${PWD}:/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/MC_Calib" \
            --volume="PATH_TO_DATA:/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/MC_Calib/V35" \
            bailool/mc-calib-prod
    '''
