import numpy as np
import os
import cv2, PIL, os
from cv2 import aruco
import operator

def checker_detection(img, aruco_dict):

    frame = cv2.imread(img)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    return len(corners)

#### INPUT ####
directory = 'D:/MY_DRIVE_N/Masters_thesis/Dataset/V20/'
new_directory = 'D:/MY_DRIVE_N/Masters_thesis/Dataset/V20_sorted/'
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
###############

camera_dict = {}
pose_dict = {}

for filename in os.listdir(directory):
    os.mkdir(os.path.join(new_directory, filename))
    camera_dict[filename] = {}
    f = os.path.join(directory, filename)
    for image in os.listdir(f):
        image_path = os.path.join(f, image)
        num_corner = checker_detection(image_path, aruco_dict)
        camera_dict[filename][image] = num_corner
        if image in pose_dict:
            pose_dict[image] += num_corner
        else:
            pose_dict[image] = num_corner

sorted_k = dict(sorted(pose_dict.items(), key=operator.itemgetter(1),reverse=True)).keys()

for filename in os.listdir(directory):
    p = 1
    for pose in sorted_k:
        frame = cv2.imread(os.path.join(directory, filename, pose))
        f = new_directory + filename + '/' + str(p)+'_' + pose
        cv2.imwrite(f, frame)
        p += 1

print('end')