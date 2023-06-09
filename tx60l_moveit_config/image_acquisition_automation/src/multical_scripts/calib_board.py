import numpy as np
import os
import cv2, PIL, os
from cv2 import aruco
import operator
from scipy.spatial.transform import Rotation as R
import glob
from src.multical.board import load_config, load_calico

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
    print(image + ' angle = ', euler_deg )

    ret2, rvecs2, tvecs2 = cv2.solvePnP(objpoints, corners, camMatrix, camDist, np.empty(1), np.empty(1))
    euler_deg2 = rotVec_to_euler(rvecs2)


    return ret, rvecs, tvecs

def rotVec_to_euler(rvecs):
    r = R.from_rotvec(rvecs.reshape(-1))
    euler_deg = r.as_euler('xyz', degrees=True)
    return euler_deg

## calibrate single camera
def cam_calib(path):
    board = board_config()
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.
    images = glob.glob(path + '/*.png')
    for fname in images:
        gray = cv2.imread(fname)[:, :, 0]
        ret, corners, ids, objp = checker_detection(gray, board)
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

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

def select_board(img_dir, boards):
    selected_board = {}
    for b in boards:
        board = boards[b].board
        images = glob.glob(img_dir + '/*.png')
        detected_corner = 0

        for fname in images:
            frame = cv2.imread(fname)
            num, corners, ids, objpoints = checker_detection(frame, board)
            detected_corner += num

        selected_board[b] = detected_corner

    board = next(iter(sorted(selected_board.items(),  reverse = False)))


    return board

if __name__ == '__main__':

    # # board = board_config()
    # boards = load_config("D:/MY_DRIVE_N/Masters_thesis/Dataset/Latest Pattern/board_images/boards.yaml")
    # img_dir = "D:/MY_DRIVE_N/Masters_thesis/Dataset/docker_V11/input/from input/08320222/"
    # # select orthonormal board for intrinsic
    # board = select_board(img_dir, boards)


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




    ##
    image = '13_p27.png'
    img_path = "D:/MY_DRIVE_N/Masters_thesis/Dataset/docker_V11/input/data/extrinsic/08320221/" + image
    frame = cv2.imread(img_path)
    board = board_config()
    num_corners, corners, ids, objpoints = checker_detection(frame, board)

    ret, rvecs, tvecs = board_pose(objpoints, corners, ids, board, cam_matrix, dist_coeff, image)

    # Draw
    imageCopy = cv2.imread(img_path)
    cv2.drawFrameAxes(imageCopy, cam_matrix, dist_coeff, rvecs, tvecs, 0.1)
    im = cv2.resize(imageCopy, (547, 364))
    cv2.imshow('out',im)
    # cv2.resizeWindow('image', (200, 200))
    cv2.waitKey(0)



    print(num_corners)