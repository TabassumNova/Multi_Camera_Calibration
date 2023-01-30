#!/usr/bin/env python3

import rospy
from src.aravis_image_acquisition import main

if __name__ == '__main__':

    rospy.init_node("Camera-42120643")
    main()
    rospy.loginfo("Camera-42120643 is now started")
    rospy.spin()

