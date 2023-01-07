import numpy as np
import cv2, PIL, os
from cv2 import aruco

allCorners = []
allIds = []
decimator = 0
# SUB PIXEL CORNER DETECTION CRITERION
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
board = aruco.CharucoBoard_create(10, 10, 0.013, 0.010, aruco_dict)

img = 'D:/MY_DRIVE_N/Masters_thesis/multical_test_new/Dataset/cam50_box3/cam2/p1.bmp'
saved_dir = r"D:/MY_DRIVE_N/Masters_thesis/multical_test_new/Dataset/cam50_box3/detected_img/p1.bmp"

frame = cv2.imread(img)
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
parameters = aruco.DetectorParameters_create()
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

for corner in corners:
    cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

# FOR DRAWING ONLY ARUCO MARKERS
frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
# cv2.imwrite(saved_dir, frame_markers)
# print("Saved image to", saved_dir)

# FOR DRAWING CHECKER CORNERS
res2 = cv2.aruco.interpolateCornersCharuco(corners,ids,gray,board)
for id, corner in zip(res2[2], res2[1]):
    cv2.drawMarker(frame, tuple([int(corner[0][0]), int(corner[0][1])]), color=(0,0,255), markerSize=10 * 2, thickness=int(1), line_type=cv2.LINE_AA)
    x, y = corner[0]
    cv2.putText(frame, str(id), (int(x + 2), int(y - 2)), cv2.FONT_HERSHEY_SIMPLEX,
                10 / 40, color=(0,0,255), lineType=cv2.LINE_AA)

cv2.imwrite(saved_dir, frame)
print("Saved image to", saved_dir)
