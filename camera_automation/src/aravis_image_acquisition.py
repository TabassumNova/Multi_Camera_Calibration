import sys
import numpy
from collections import namedtuple

from .aravis import Camera
import cv2

def arv_image_acquisition(cam_model, exposure_time, path):
    cam = Camera(cam_model)
    cam.set_feature("Width", 5472)
    cam.set_feature("Height", 3648)
    cam.set_frame_rate(2)
    cam.set_exposure_time(exposure_time)
    #cam.start_acquisition_trigger()
    cam.start_acquisition_continuous()
    try:
        
        #cam.trigger()
        frame = cam.pop_frame()
        print("Saving image to ", path)
        cv2.imwrite(path, frame)
    finally:
        cam.stop_acquisition()

    