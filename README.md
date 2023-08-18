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
    

## Camera Aravis python
- pip install pyobs-aravis
- https://github.com/SintefManufacturing/python-aravis/blob/master/aravis.py
- https://github.com/SintefManufacturing/python-aravis
- https://github.com/SintefManufacturing/python-aravis/blob/5750250cedb9b96d7a0172c0da9c1811b6b817af/examples/save-image.py

## Camera Aravis ROS Package
- Add this repo (https://github.com/FraunhoferIOSB/camera_aravis) in your catkin workspace 
- catkin_make

### ROS camera aravis application
- Run `roscore`
- To check which camera models are connected, run from another terminal `arv-tool-0.4`.
    Output: 
    ```
    The Imaging Source Europe GmbH-11120229
    The Imaging Source Europe GmbH-42120643
    ```
- To run it in a given namespace 
`ROS_NAMESPACE=cam1 rosrun camera_aravis cam_aravis _guid:=GmbH-11120229`
- From another terminal, run `ROS_NAMESPACE=cam2 rosrun camera_aravis cam_aravis _guid:=GmbH-42120643`

# ROS wrapper for image acquisition automation
- Create ROS package (http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
```
cd tx60_moveit/src
catkin_create_pkg camera_automation std_msgs rospy roscpp
catkin_make
```
- Create python node (http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
- catkin_make
- chmod +x ros_wrapper.py
- Inside launch file 
```
<node pkg="xsens_driver" name="mtnode" type="mtnode.py" output="screen"> 
</node>
```
#### Issues
- `/usr/bin/env: ‘python3\r’: No such file or directory` : 
    - sudo apt install dos2unix
    - dos2unix python_file.py

# GUI
## Paraview
- https://www.youtube.com/watch?v=FTUBpqkC3Ss
- To add python shell in paraview application: view > Python shell
- Python program can be run from command prompt: 
    - To enable this, add the path of pvpython.exe 'Program Files/ParaView 5.11.0/bin' in environment variable
    - From cmd, run `pvpython.exe`
    - for loading imagefile `reader = OpenDataFile("D:/path_to_the_folder/group01_img1.png")`
    - Or write a python script and run from cmd `pvpython.exe D:/path_to_python_script/parav3.py`

## pyvistaqt
- https://qtdocs.pyvista.org/usage.html
- https://www.pythonguis.com/faq/adding-images-to-pyqt5-applications/
- Convert pyQt UI to python: command on shell: pyuic5.exe input.ui -o output.py

## Qt Designer
- For loading image-> label > in Pixmap, load image > scaledContents check

## Aruco board Create
- https://gitlab.lrz.de/autotron-group/camera_calib_nova/-/blob/main/tx60l_moveit_config/image_acquisition_automation/src/printing%20board/write_board2.py
- Parameters 
```
--rows
9
--columns
12
-T
charuco_board
--square_size
13
--marker_size
8
-f
DICT_6X6_1000.json
```


# Directory description
## [tx60_moveit_config](https://gitlab.lrz.de/autotron-group/camera_calib_nova/-/tree/main/tx60l_moveit_config)
- Contains robot motion for image acquisition
- Git clone this repo in your workspace
- From terminal run `roslaunch tx60l_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.0.250`
- From another terminal run `rosrun tx60l_moveit_config main.py`

# Camera Parameters
- Intrinsic (https://1drv.ms/f/s!At7UW7zuEoCCivQPV5hxq03VmbaI4A?e=VPgsl9)

*** Not all the views are taken for calculating intrinsic. The program iteratively rejects outliers that have greater reprojection erro and optimizes the intrinsic parameters. How many views are taken for calculation are written in calibration.txt file

**** cam222 has only 28 views??

# OpenCV Issues
- For aruco, PyQT related issues
    - pip3 uninstall opencv-python
    - python3 -m pip install opencv-contrib-python
    - pip install opencv-contrib-python-headless==4.6.0.66
- cv2.solvePnPGeneric sometimes show irrelevent results
    - take 3/5 pictures of the same poses. Calculate pose. Take average of the poses

- openCV >= 4.6 (Python>= 3.6): 
    - solvePnPGeneric/ solvePnP working
    - PyQt5 not working
- openCV = 4.2:
    - solvePnPGeneric not working
    - solvePnP working
    - PyQt5 working

