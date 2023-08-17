import sys
import numpy as np
from collections import namedtuple
from .helpers import make_directory
from .aravis import Camera, get_device_ids
import cv2

def find_cameras():
    cameras = get_device_ids()
    num_cams = len(cameras)
    camera_serial = []
    for cam_idx in range(num_cams):
        serial = cameras[cam_idx][-8:-1]+cameras[cam_idx][-1]
        camera_serial.append(int(serial))

    return num_cams, camera_serial


def show_image():
    cameras = get_device_ids()
    num_cams = len(cameras)
    # arv_camera = []
    camera_serial = []
    # base_path = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/'
    # new_path = []
    cam_images = {}

    for cam_idx in range(num_cams):
        serial = int(cameras[cam_idx][-8:-1]+cameras[cam_idx][-1])
        # camera_serial.append(serial)
        cam = Camera(cameras[cam_idx])
        cam.set_feature("Width", 5472/2)
        cam.set_feature("Height", 3648/2)
        cam.set_frame_rate(2)
        cam.set_exposure_time(1000000)
        # arv_camera.append(cam)
        # new_path.append(base_path + camera_serial[cam_idx] + '/')
        # make_directory(new_path[cam_idx])
        cam.start_acquisition_continuous()
        frame = np.asarray(cam.pop_frame())
        cam_images[serial] = frame
        # print(cam_images[serial].max)
        cam.stop_acquisition()

    return cam_images

    # while True:
    #     # multi cam: jump from camera to camera
    #     for cam_idx in range(num_cams):
    #         cam = arv_camera[cam_idx]
    #         cam.start_acquisition_continuous()
    #         try:
    #             saved_path = new_path[cam_idx] + '/p' + str(pose) + '.png'
    #             frame = cam.pop_frame()
    #             cv2.imwrite(saved_path, frame)

    #         except:
    #             print("[ERROR] Aborting publishing")
    #             exit(-1)
    #         cam.stop_acquisition()
    #     if cam_idx == num_cams - 1:
    #         break

    