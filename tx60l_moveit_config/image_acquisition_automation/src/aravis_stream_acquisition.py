# https://github.com/SintefManufacturing/python-aravis/blob/5750250cedb9b96d7a0172c0da9c1811b6b817af/examples/streaming-cv.py

import time
import sys
import numpy
from collections import namedtuple
from .helpers import make_directory
from .aravis import Camera, get_device_ids
import cv2

path = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/'

def start_streaming(path):
    cameras = get_device_ids()
    num_cams = len(cameras)
    arv_camera = []
    camera_serial = []
    base_path = path
    new_path = []

    for cam_idx in range(num_cams):
        serial = cameras[cam_idx][-8:-1]+cameras[cam_idx][-1]
        camera_serial.append(serial)
        cam = Camera(cameras[cam_idx])
        cam.set_feature("Width", 5472)
        cam.set_feature("Height", 3648)
        cam.set_frame_rate(2)
        # cam.set_exposure_time(2000000)
        arv_camera.append(cam)
        new_path.append(base_path + camera_serial[cam_idx] + '/')
        make_directory(new_path[cam_idx])

    try:
        # multi cam: jump from camera to camera
        for cam_idx in range(num_cams):
            cam = arv_camera[cam_idx]
            cam.start_acquisition_continuous()
            # cv2.namedWindow('capture', flags=0)
            pose = 0
            while True:
                cam.start_acquisition_continuous()
                saved_path = new_path[cam_idx] + '/p' + str(pose) + '.png'
                frame = cam.pop_frame()
                # if not 0 in frame.shape:
                #     cv2.imshow("capture", frame)
                #     cv2.waitKey(10)
                cv2.imwrite(saved_path, frame)
                print('Pose: ', pose)
                pose += 1
                time.sleep(15)
                cam.stop_acquisition()

    finally:
        cam.stop_acquisition()



# if __name__ == "__main__":
#     start_streaming(path)