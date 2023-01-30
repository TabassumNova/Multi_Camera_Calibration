import sys
import numpy
from collections import namedtuple

from .aravis import Camera
import cv2

def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        path = "frame.png"
    #cam = aravis.Camera("AT-Automation Technology GmbH-20805103")
    # cam = Aravis.Camera('The Imaging Source Europe GmbH-42120643')
    cam = Camera('The Imaging Source Europe GmbH-11120241')
    cam.set_feature("Width", 5472)
    cam.set_feature("Height", 3648)
    cam.set_frame_rate(2)
    cam.set_exposure_time(2000000)
    #cam.start_acquisition_trigger()
    cam.start_acquisition_continuous()
    try:
        path = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/arv_im.png'
        #cam.trigger()
        frame = cam.pop_frame()
        print("Saving image to ", path)
        cv2.imwrite(path, frame)
    finally:
        cam.stop_acquisition()

    