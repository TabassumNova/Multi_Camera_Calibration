import numpy as np
import os
import cv2, PIL, os
from cv2 import aruco
import operator
from scipy.spatial.transform import Rotation as R
import math
import src.multical.app.boards as boards
import json
import yaml
from src.multical import board

from src.multical.transform.matrix import *
from src.multical.transform.rtvec import *

def board_config(detected_board):
    dict_name = detected_board.aruco_dict
    dict_id = dict_name if isinstance(dict_name, int) \
        else getattr(cv2.aruco, f'DICT_{dict_name}')
    # dict_id =
    offset = detected_board.aruco_offset
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    width, height = detected_board.size
    square_length, marker_length = detected_board.square_length, detected_board.marker_length
    aruco_dict.bytesList = aruco_dict.bytesList[offset:]
    board = cv2.aruco.CharucoBoard_create(width, height, square_length, marker_length, aruco_dict)
    return board


def board_pose(objpoints, corners, ids, board, camMatrix, camDist, image):
    # ret, rvecs, tvecs = cv2.aruco.estimatePoseCharucoBoard(corners, ids, board, camMatrix, camDist, np.empty(1), np.empty(1))
    # euler_deg = rotVec_to_euler(rvecs)
    # print(image + ' angle = ', euler_deg )

    if ids.shape[0]<6:
        print('Not enough points to calculate solvePnP')
        ret2, rvecs2, tvecs2 = 0, np.zeros((3,1)), np.zeros((3,1))
        euler_deg2 = np.zeros((3,1))
    else:
        ret2, rvecs2, tvecs2 = cv2.solvePnP(objpoints, corners, camMatrix, camDist, np.empty(1), np.empty(1))
        euler_deg2 = rotVec_to_euler(rvecs2)

    return ret2, rvecs2.reshape(-1), tvecs2.reshape(-1), euler_deg2

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

def draw(frame, board_num, euler_deg, tvecs, text_height):
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Draw_cube
    # project 3D points to image plane
    axis = np.float32([[3 * .013, 0, 0], [0, 3 * .013, 0], [0, 0, -3 * .013]]).reshape(-1, 3)
    axis_cube = np.float32([[0, 0, 0], [0, 3 * 0.013, 0], [3 * 0.013, 3 * 0.013, 0], [3 * 0.013, 0, 0],
                            [0, 0, -3 * 0.013], [0, 3 * 0.013, -3 * 0.013], [3 * 0.013, 3 * 0.013, -3 * 0.013],
                            [3 * 0.013, 0, -3 * 0.013]])
    imgpts, jac = cv2.projectPoints(axis_cube, rvecs, tvecs, cam_matrix, cam_dist)
    frame = draw_cube(frame, np.int_(np.floor(corners)), np.int_(np.floor(imgpts)))

    cv2.putText(frame, 'Board' + str(board_num), (10, text_height), font, 2, (0, 0, 255), 5, cv2.LINE_AA)
    cv2.putText(frame, 'Euler_degree ' + str(euler_deg), (10, text_height + 100), font, 2, (0, 0, 255), 5, cv2.LINE_AA)
    cv2.putText(frame, 'tvecs ' + str(tvecs), (10, text_height + 200), font, 2, (0, 0, 255), 5, cv2.LINE_AA)
    # cv2.drawFrameAxes(frame, cam_matrix, cam_dist, rvecs, tvecs, 0.1)
    start_height = text_height + 400
    return frame, text_height

def get_base_gripper_transformation(end_effector_poses, filename):
    pose_name = filename[:-4]
    pose_num = pose_name[1:]
    EF_file = open(end_effector_poses)
    EF_poses = json.load(EF_file)
    pose = EF_poses[pose_num]
    position = np.array([float(p) for p in pose['position (x,y,z)']])
    orientation = [float(o) for o in pose['orintation (x,y,z)']]
    r = R.from_euler('xyz', orientation, degrees=True)
    rot_matrix = r.as_matrix()

    return rot_matrix, position


if __name__ == '__main__':
    directory = 'D:\MY_DRIVE_N\Masters_thesis\Dataset\geometrical_method/08320218/'
    new_directory = 'D:\MY_DRIVE_N\Masters_thesis\Dataset\geometrical_method/08320217_angle/'
    board_yaml = "D:\MY_DRIVE_N\Masters_thesis\Dataset\geometrical_method/boards.yaml"
    camera_intrinsic = "D:\MY_DRIVE_N\Masters_thesis\Dataset\geometrical_method/all_camera_intrinsic.json"
    end_effector_poses = "D:\MY_DRIVE_N\Masters_thesis\Dataset\geometrical_method/poses_geo.json"
    hand_eye = "D:\MY_DRIVE_N\Masters_thesis\Dataset\geometrical_method/hand_eye.json"

    camera_file = open(camera_intrinsic)
    cameras = json.load(camera_file)
    cam_name = directory[-9:-1]
    cam_matrix = np.array(cameras['cameras'][cam_name]['K'])
    cam_dist = np.array(cameras['cameras'][cam_name]['dist'])

    board_config_file = board.load_config(board_yaml)
    base_gripper = {}
    world_camera = {}
    base_gripper['R'] = []
    base_gripper['T'] = []
    world_camera['R'] = []
    world_camera['T'] = []
    json_dict = {}
    load_hande_eye = False
    if load_hande_eye == False:
        for filename in os.listdir(directory):
            print(filename)
            image_path = os.path.join(directory, filename)
            frame = cv2.imread(image_path)
            b = boards.Boards(boards=board_yaml, detect=image_path, pixels_mm=10, show_image=False)
            # b = boards.Boards(boards=board_yaml, detect=detect_img)
            detection = b.execute()
            detected_boards = [idx for idx,d in enumerate(detection) if len(d.ids)>0]
            text_height = 200
            base_gripper_R, base_gripper_T = get_base_gripper_transformation(end_effector_poses, filename)
            for idx,board_num in enumerate(detected_boards):
                ids = detection[board_num].ids
                corners = detection[board_num].corners
                detected_board = board_config_file[list(board_config_file)[board_num]]
                adjusted_points = detected_board.adjusted_points
                objpoints = np.array([adjusted_points[a] for a in ids], dtype='float64').reshape((-1, 3))
                detect_board_config = board_config(detected_board)
                ret, rvecs, tvecs, euler_deg = board_pose(objpoints, corners, ids, detect_board_config, cam_matrix, cam_dist, frame)
                if ret:
                    rtvecs = join(rvecs, tvecs)
                    rtmatrix = np.linalg.inv(to_matrix(rtvecs))
                    R_matrix, T = matrix.split(rtmatrix)
                    world_camera['R'].append(R_matrix)
                    world_camera['T'].append(T)
                    base_gripper['R'].append(base_gripper_R)
                    base_gripper['T'].append(base_gripper_T)
                    # frame, text_height = draw(frame, board_num, euler_deg, tvecs, text_height)
            pass
            # cv2.imwrite(image_path, frame)
            pass
        gripper_world_r, gripper_world_t, base_cam_r, base_cam_t = \
            cv2.calibrateRobotWorldHandEye(
                world_camera['R'], world_camera['T'],
                base_gripper['R'], base_gripper['T'])

        gripper_world_matrix = matrix.join(gripper_world_r, gripper_world_t.reshape(-1))
        base_cam_matrix = matrix.join(base_cam_r, base_cam_t.reshape(-1))
        json_dict['gripper_world_matrix'] = gripper_world_matrix.tolist()
        json_dict['base_cam_matrix'] = base_cam_matrix.tolist()
        json_object = json.dumps(json_dict, indent=4)
        # Writing to sample.json
        with open(hand_eye, "w") as outfile:
            outfile.write(json_object)

    else:
        hand_eye_json_file = open(hand_eye)
        hand_eye_json = json.load(hand_eye_json_file)
        gripper_world_matrix = hand_eye_json['gripper_world_matrix']
        base_cam_matrix = hand_eye_json['base_cam_matrix']

    ## Reprojection_error (Repeatative_change this part)
    error_dict = {}
    face = 1
    for filename in os.listdir(directory):
        print(filename)
        image_path = os.path.join(directory, filename)
        frame = cv2.imread(image_path)
        b = boards.Boards(boards=board_yaml, detect=image_path, pixels_mm=10, show_image=False)
        # b = boards.Boards(boards=board_yaml, detect=detect_img)
        detection = b.execute()
        detected_boards = [idx for idx, d in enumerate(detection) if len(d.ids) > 0]
        text_height = 200
        base_gripper_R, base_gripper_T = get_base_gripper_transformation(end_effector_poses, filename)
        base_gripper_matrix = matrix.join(base_gripper_R, base_gripper_T)
        for idx, board_num in enumerate(detected_boards):
            ids = detection[board_num].ids
            corners = detection[board_num].corners
            detected_board = board_config_file[list(board_config_file)[board_num]]
            adjusted_points = detected_board.adjusted_points
            objpoints = np.array([adjusted_points[a] for a in ids], dtype='float64').reshape((-1, 3))
            detect_board_config = board_config(detected_board)
            ret, rvecs, tvecs, euler_deg = board_pose(objpoints, corners, ids, detect_board_config, cam_matrix,
                                                      cam_dist, frame)
            if ret:
                rtvecs = join(rvecs, tvecs)
                rtmatrix = to_matrix(rtvecs)
                # R_matrix, T = matrix.split(rtmatrix)
                final_transformation = (base_cam_matrix) @ base_gripper_matrix @ np.linalg.inv(gripper_world_matrix)
                r_final, t_final = split(from_matrix(final_transformation))
                imagePoints, _ = cv2.projectPoints(objpoints, r_final, t_final, cam_matrix, cam_dist)
                error = imagePoints.reshape([-1, 2]) - corners

                error_dict[face] = error
                face += 1
                pass

    pass
