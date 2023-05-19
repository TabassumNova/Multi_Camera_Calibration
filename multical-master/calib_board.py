import numpy as np
import os
import cv2, PIL, os
from cv2 import aruco
import operator

def checker_detection(img, aruco_dict):

    frame = cv2.imread(img)
    attrs = {'adaptiveThreshWinSizeMax': 200, 'adaptiveThreshWinSizeStep': 50}
    parameters = aruco.DetectorParameters_create()
    for k, v in attrs.items():
        assert hasattr(parameters, k), f"aruco_config: no such detector parameter {k}"
        setattr(parameters, k, v)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    return corners

dict_id = 11
offset = 0
aruco_dict=cv2.aruco.getPredefinedDictionary(dict_id)
aruco_dict.bytesList=aruco_dict.bytesList[offset:]

img_path = 'D:\MY_DRIVE_N\Masters_thesis\Dataset\docker_V11\input\data/08320218/8_p76.png'
corners = checker_detection(img_path, aruco_dict)

# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# corners2 = cv2.cornerSubPix(gray,corners,(4,4),(-1,-1),criteria)

# Draw and display the corners
# im = cv2.imread(img_path)
# img = cv2.drawChessboardCorners(im, (12, 9), corners, True)
# if(True):
#     #cv2.namedWindow('image',cv2.WINDOW_NORMAL)
#     im = cv2.resize(img, (1000,1000))
#     cv2.imshow('img',im)
#     #cv2.resizeWindow('image', (200,200))
#     cv2.waitKey(0)


print(np.array(corners))