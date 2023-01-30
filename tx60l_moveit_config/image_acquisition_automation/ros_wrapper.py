#!/usr/bin/env python

import rospy
from src.TIS_Cam import TIS, TIS_list_devices

if __name__ == '__main__':

    rospy.init_node("Cameras")
    cam1 = TIS("11120237", 1920, 1080, 15, 1, False)
    cam2 = TIS("11120241", 1920, 1080, 15, 1, False)

    image1 = cam1.Get_image()
    image2 = cam2.Get_image()

    # rospy.on_shutdown(cam1.Get_image())
    # rospy.on_shutdown(cam2.Get_image())
    rospy.loginfo("Cameras are now started")
    rospy.spin()

