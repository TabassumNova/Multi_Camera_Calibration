#!/usr/bin/env python3

import sys
import rospy
from src.aravis_image_acquisition import arv_image_acquisition

# Command
# rosrun camera_automation cam_node.py 42120643 2000000 /home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/arv_im.png 

if __name__ == '__main__':

    if len(sys.argv) > 1:
        myargv = rospy.myargv(argv=sys.argv)
    else:
        myargv = []

    if len(myargv) == 2:
        cam_model = 'The Imaging Source Europe GmbH-' + str(myargv[1])
    elif len(myargv) == 3:
        cam_model = str(myargv[1])
        exposure_time = myargv[2]
    elif len(myargv) == 4:
        cam_model = 'The Imaging Source Europe GmbH-' + str(myargv[1])
        exposure_time = myargv[2]
        path = str(myargv[3])
    else:
        cam_model = 'The Imaging Source Europe GmbH-42120643'
        exposure_time = 2000000
        path = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/arv_im.png'

    if len(myargv) > 1:
        node = 'Camera-' + str(myargv[1])
    else:
        node = 'Camera'

    
    rospy.init_node(node)
    # rospy.loginfo(cam_model, exposure_time, path)
    arv_image_acquisition(str(cam_model), int(exposure_time), str(path))
    rospy.loginfo(node + " is now started")
    rospy.spin()

