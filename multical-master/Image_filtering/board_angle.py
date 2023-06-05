import numpy as np
import os
import cv2, PIL, os
from cv2 import aruco
import operator
from scipy.spatial.transform import Rotation as R
import math

def board_config():
    ## set input
    cam_id = '08320217'
    dict_id = 11
    offset = 216
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    width, height = (12, 9)
    square_length, marker_length = 0.013, 0.009
    aruco_dict.bytesList = aruco_dict.bytesList[offset:]
    board = cv2.aruco.CharucoBoard_create(width, height, square_length, marker_length, aruco_dict)
    return board

def checker_detection(frame, board):

    attrs = {'adaptiveThreshWinSizeMax': 200, 'adaptiveThreshWinSizeStep': 50}
    parameters = aruco.DetectorParameters_create()
    for k, v in attrs.items():
        assert hasattr(parameters, k), f"aruco_config: no such detector parameter {k}"
        setattr(parameters, k, v)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, board.dictionary, parameters=parameters)
    if corners:
        num, corners, ids = cv2.aruco.interpolateCornersCharuco(corners, ids, frame, board)
        # corners = np.array(corners, dtype='float64').reshape((-1,2))
        objp = board.chessboardCorners
        objpoints = np.array([objp[i] for i in ids], dtype='float64').reshape((-1,3))
    else:
        num, corners, ids, objpoints = 0, 0, 0, 0

    return num, corners, ids, objpoints

def board_pose(objpoints, corners, ids, board, camMatrix, camDist, image):
    ret, rvecs, tvecs = cv2.aruco.estimatePoseCharucoBoard(corners, ids, board, camMatrix, camDist, np.empty(1), np.empty(1))
    euler_deg = rotVec_to_euler(rvecs)
    # print(image + ' angle = ', euler_deg )

    ret2, rvecs2, tvecs2 = cv2.solvePnP(objpoints, corners, camMatrix, camDist, np.empty(1), np.empty(1))
    euler_deg2 = rotVec_to_euler(rvecs2)

    return ret, rvecs, tvecs, euler_deg

def rotVec_to_euler(rvecs):
    r = R.from_rotvec(rvecs.reshape(-1))
    euler_deg = r.as_euler('xyz', degrees=True)
    return euler_deg

def draw_cube(img, corners, imgpts):
    img = np.float64(img)
    imgpts = np.int32(imgpts).reshape(-1,2)
    # draw ground floor in green
    img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)
    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
    # draw top layer in red color
    img = cv2.drawContours(img, [imgpts[4:]],-1,(0,0,255),3)
    return img

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = np.float64(img)
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

#### INPUT ####
directory = 'D:/MY_DRIVE_N/Masters_thesis/Dataset/board_angle_analysys/amy_dts/'
new_directory = 'D:/MY_DRIVE_N/Masters_thesis/Dataset/board_angle_analysys/output/'
board = board_config()
###############
cam_matrix = np.array([
        [
          21573.93062447028,
          0.0,
          2735.1256025391835
        ],
        [
          0.0,
          21613.39963789188,
          1823.9257586838949
        ],
        [
          0.0,
          0.0,
          1.0
        ]
      ])
dist_coeff = np.array([
    [
      0.07350996335092069,
      -3.766971505779897,
      -0.0066301624992787045,
      0.0009685106550848963,
      -0.053382230338634616
    ]
  ])
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

for filename in os.listdir(directory):
    # os.mkdir(os.path.join(new_directory, filename))
    # camera_dict[filename] = {}
    input_path = os.path.join(directory, filename)
    # for image in os.listdir(f):
    # image_path = os.path.join(f, image)
    frame = cv2.imread(input_path)
    num_corners, corners, ids, objpoints = checker_detection(frame, board)
    ret, rvecs, tvecs, euler_deg = board_pose(objpoints, corners, ids, board, cam_matrix, dist_coeff, frame)
    corners2 = cv2.cornerSubPix(frame[:,:,0], corners, (11, 11), (-1, -1), criteria)
    output_path = new_directory + filename
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Draw_cube
    # project 3D points to image plane
    axis = np.float32([[3*.013,0,0], [0,3*.013,0], [0,0,-3*.013]]).reshape(-1,3)
    axis_cube = np.float32([[0, 0, 0], [0, 3*0.013, 0], [3*0.013, 3*0.013, 0], [3*0.013, 0, 0],
                       [0, 0, -3*0.013], [0, 3*0.013, -3*0.013], [3*0.013, 3*0.013, -3*0.013], [3*0.013, 0, -3*0.013]])
    imgpts, jac = cv2.projectPoints(axis_cube, rvecs, tvecs, cam_matrix, dist_coeff)
    frame = draw_cube(frame, np.int_(np.floor(corners2)), np.int_(np.floor(imgpts)))

    cv2.putText(frame, str(euler_deg), (10, 400), font, 2, (0, 0, 255), 5, cv2.LINE_AA)
    cv2.drawFrameAxes(frame, cam_matrix, dist_coeff, rvecs, tvecs, 0.1)
    cv2.imwrite(output_path, frame)

print('end')