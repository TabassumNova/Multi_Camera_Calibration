# Camera_Calib_Nova

-  `/multical-master/camcalib_calibrate.py` replaces command `multical calibrate --image_path D:\MY_DRIVE_N\Masters_thesis\multical_test_new\net-2-base\data`
- `/multical-master/camcalib_detect.py` replaces command `multical boards --boards my_board.yaml --detect my_image.jpeg`
- `/test_files/calibration.py` --> opencv calibration using checker board images
- `/test_files/charuco_detect.py` --> opencv charuco detection

## 20 MP TIS camera original software
- https://www.theimagingsource.de/support/software-f%C3%BCr-linux/
- It is installed with "sudo apt install .​/​tiscamera_0.n.n_amd64_ubuntu_1804.deb". "n" is to be replaced by the corresponding version number.
- After installation, start the "tcam-capture" program in the terminal to use a camera.
- Code example in https://github.com/TheImagingSource/tiscamera/tree/master/examples

## Camera Aravis API
- For installation follow this link: https://aravisproject.github.io/aravis/building.html
- Steps: 
    - At first, download source code from https://github.com/AravisProject/aravis/releases
    - Unzip and from terminal navigate to the folder
    - Download dependencies for Ubuntu 20.04
    ```
    sudo apt install libxml2-dev libglib2.0-dev cmake libusb-1.0-0-dev gobject-introspection \
                 libgtk-3-dev gtk-doc-tools  xsltproc libgstreamer1.0-dev \
                 libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev \
                 libgirepository1.0-dev gettext
    
    pip3 install --user meson==0.56.0
    ```
    - Install
```
    meson setup build
    cd build
    ninja
    ninja install
    sudo ldconfig
```
### Alternative way to install API
- Follow this method (https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-aravis.md)
- Steps: 
    - Download a newer version from (https://download.gnome.org/sources/aravis/0.4/)
    - Uncompress the downloaded Aravis package and open a terminal in that folder.
    - Run `./configure`
    - Run `make`
    - Run `sudo make install`
    
## Camera Aravis ROS Package
- Add this repo (https://github.com/FraunhoferIOSB/camera_aravis) in your catkin workspace 
- catkin_make

### ROS camera aravis application
- Run `roscore`
- To check which camera models are connected `arv-tool-0.4`
- 
