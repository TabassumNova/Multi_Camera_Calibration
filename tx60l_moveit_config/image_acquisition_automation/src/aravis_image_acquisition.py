import sys
import numpy
from collections import namedtuple
from .helpers import make_directory
from .aravis import Camera, get_device_ids
import cv2

def start_arv_image_acquisition(self, pose):
    cameras = get_device_ids()
    num_cams = len(cameras)
    arv_camera = []
    camera_serial = []
    base_path = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/'
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

    while True:
        # multi cam: jump from camera to camera
        for cam_idx in range(num_cams):
            cam = arv_camera[cam_idx]
            cam.start_acquisition_continuous()
            try:
                saved_path = new_path[cam_idx] + '/p' + str(pose) + '.png'
                frame = cam.pop_frame()
                cv2.imwrite(saved_path, frame)

            except:
                print("[ERROR] Aborting publishing")
                exit(-1)
            cam.stop_acquisition()
        if cam_idx == num_cams - 1:
            break

def arv_get_image(path, pose):
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

    while True:
        # multi cam: jump from camera to camera
        for cam_idx in range(num_cams):
            cam = arv_camera[cam_idx]
            cam.start_acquisition_continuous()
            try:
                saved_path = new_path[cam_idx] + '/p' + str(pose) + '.png'
                frame = cam.pop_frame()
                cv2.imwrite(saved_path, frame)

            except:
                print("[ERROR] Aborting publishing")
                exit(-1)
            cam.stop_acquisition()
        if cam_idx == num_cams - 1:
            break

    return camera_serial

    