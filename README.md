# Camera_Calib_Nova

-  `/multical-master/camcalib_calibrate.py` replaces command `multical calibrate --image_path D:\MY_DRIVE_N\Masters_thesis\multical_test_new\net-2-base\data`
- `/multical-master/camcalib_detect.py` replaces command `multical boards --boards my_board.yaml --detect my_image.jpeg`
- `/test_files/calibration.py` --> opencv calibration using checker board images
- `/test_files/charuco_detect.py` --> opencv charuco detection

## 20 MP TIS camera original software
- https://www.theimagingsource.de/support/software-f%C3%BCr-linux/
- It is installed with "sudo apt install .​/​tiscamera_0.n.n_amd64_ubuntu_1804.deb". "n" is to be replaced by the corresponding version number.
- After installation, start the "tcam-capture" program in the terminal to use a camera.
- Code example in https://github.com/TheImagingSource/tiscamera

