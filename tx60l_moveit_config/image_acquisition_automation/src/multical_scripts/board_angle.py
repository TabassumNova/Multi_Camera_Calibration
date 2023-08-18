
import cv2, PIL, os

import src.multical.app.boards as boards
import numpy as np

import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
from src.multical.tables import *

import json

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


def board_pose(objpoints, corners, ids, camMatrix, camDist, method="solvePnPGeneric"):
    # ret, rvecs, tvecs = cv2.aruco.estimatePoseCharucoBoard(corners, ids, board, camMatrix, camDist, np.empty(1), np.empty(1))
    # euler_deg = rotVec_to_euler(rvecs)
    # print(image + ' angle = ', euler_deg )

    if ids.shape[0]<6:
        print('Not enough points to calculate solvePnP')
        ret, rvecs, tvecs = 0, np.zeros((3,1)), np.zeros((3,1))
        euler_deg = np.zeros((3,1))
        view_angle = 0
    elif method == "solvePnP":
        ret, rvecs, tvecs = cv2.solvePnP(objpoints, corners, camMatrix, camDist, np.empty(1), np.empty(1))
        euler_deg = rotVec_to_euler(rvecs)
        view_angle = view_angle_calc(rvecs)
    elif method == "solvePnPGeneric":
        r = np.array([], dtype=np.float32)
        ret, rvecs, tvecs, error = cv2.solvePnPGeneric(objpoints, corners, camMatrix, camDist)
        rvecs = rvecs[0]
        tvecs = tvecs[0]
        euler_deg = rotVec_to_euler(rvecs)
        view_angle = view_angle_calc(rvecs)
    elif method == "solvePnP_P3P":
        ret, rvecs, tvecs = cv2.solveP3P(objpoints[0:3,:], corners[0:3,:], camMatrix, camDist, flags=cv2.SOLVEPNP_P3P)
        if ret:
            rvecs = rvecs[0]
            tvecs = tvecs[0]
            euler_deg = rotVec_to_euler(rvecs)
            view_angle = view_angle_calc(rvecs)
        else:
            ret, rvecs, tvecs = 0, np.zeros((3, 1)), np.zeros((3, 1))
            euler_deg = np.zeros((3, 1))
            view_angle = 0

    return ret, rvecs.reshape(-1), tvecs.reshape(-1), euler_deg, view_angle

def view_angle_calc(rvec):
    r = R.from_rotvec(rvec.reshape(-1))
    x,y,z,w = r.as_quat()
    view_angle = math.degrees(np.arctan((2*(x*y - w*z)*w*w - x*x -y*y +z*z)))
    return view_angle

def rotVec_to_euler(rvecs):
    r = R.from_rotvec(rvecs.reshape(-1))
    euler_deg = r.as_euler('xyz', degrees=True)
    return euler_deg

def euler_to_rotVec(euler_angle):
    r = R.from_euler('xyz', euler_angle, degrees=True)
    rvec = r.as_rotvec()
    return rvec

def draw_cube(img, imgpts):
    img = np.float64(img)
    imgpts = np.int32(imgpts).reshape(-1,2)
    # draw ground floor in green
    # img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)
    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
    # draw top layer in red color
    img = cv2.drawContours(img, [imgpts[4:]],-1,(0,0,255),3)
    return img

def draw(frame, board_num, euler_deg, rvecs, tvecs, cam_matrix, cam_dist, text_height):
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Draw_cube
    # project 3D points to image plane
    axis = np.float32([[3 * .013, 0, 0], [0, 3 * .013, 0], [0, 0, -3 * .013]]).reshape(-1, 3)
    axis_cube = np.float32([[0, 0, 0], [0, 3 * 0.013, 0], [3 * 0.013, 3 * 0.013, 0], [3 * 0.013, 0, 0],
                            [0, 0, -3 * 0.013], [0, 3 * 0.013, -3 * 0.013], [3 * 0.013, 3 * 0.013, -3 * 0.013],
                            [3 * 0.013, 0, -3 * 0.013]])
    imgpts, jac = cv2.projectPoints(axis_cube, rvecs, tvecs, cam_matrix, cam_dist)
    # frame = draw_cube(frame, np.int_(np.floor(imgpts)))

    cv2.putText(frame, 'Board' + str(board_num), (10, text_height), font, 2, (0, 0, 255), 5, cv2.LINE_AA)
    cv2.putText(frame, 'Euler_degree ' + str(euler_deg), (10, text_height + 100), font, 2, (0, 0, 255), 5, cv2.LINE_AA)
    cv2.putText(frame, 'tvecs ' + str(tvecs), (10, text_height + 200), font, 2, (0, 0, 255), 5, cv2.LINE_AA)
    cv2.drawFrameAxes(frame, cam_matrix, cam_dist, rvecs, tvecs, 0.1)
    start_height = text_height + 600
    return frame, start_height

def draw_marker_pose(frame, ids, corners, marker_length, cam_matrix, cam_dist):

    if len(corners) > 0:
        for i in range(0, len(ids)):
            x = (corners[i]).astype('float32')
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(x, marker_length, cam_matrix, cam_dist)
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners)

            # Draw Axis
            cv2.drawFrameAxes(frame, cam_matrix, cam_dist, rvec, tvec, 0.01, thickness=10)

    return frame

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

def draw_EF_axis(img, R, t, K, dist):
    # unit is mm
    rotV = R
    points = np.float32([[3 * .013, 0, 0], [0, 3 * .013, 0], [0, 0, -3 * .013], [0, 0, 0]]).reshape(-1, 3)
    # points = np.float32([[0.013, 0, 0], [0, 0.013, 0], [0, 0, 0.013], [0, 0, 0]]).reshape(-1, 3)
    axisPoints, _ = cv2.projectPoints(points, rotV, np.array([0.0, 0.0, 0.0]), K, dist)
    axisPoints = np.int_(np.floor((axisPoints.reshape((-1,2)))/3))
    x0, x1 = line_extend(axisPoints[3], axisPoints[0])
    y0, y1 = line_extend(axisPoints[3], axisPoints[1])
    z0, z1 = line_extend(axisPoints[3], axisPoints[2])
    img = cv2.line(img, x0, x1, (255,0,0), 10)
    img = cv2.line(img, y0, y1, (0,255,0), 10)
    img = cv2.line(img, z0, z1, (0,0,255), 10)
    cv2.imwrite(image_path, img)
    return img

def line_extend(start, end):
    slope = (end[1] - start[1])/ (end[0] - start[0])
    e = end[0]*1.5*slope
    end1 = abs(np.int_((end[0]*1.5, end[0]*1.5*slope)))
    start1 = abs(np.int_(start))

    return tuple(start1), tuple(end1)


def write_base_gripper_pose(frame, end_effector_poses, filename, text_height):
    font = cv2.FONT_HERSHEY_SIMPLEX
    pose_name = filename[:-4]
    pose_num = pose_name[1:]
    EF_file = open(end_effector_poses)
    EF_poses = json.load(EF_file)
    pose = EF_poses[pose_num]
    position = np.array([float(p) for p in pose['position (x,y,z)']])
    orientation = [float(o) for o in pose['orintation (w,x,y,z)']]
    r = R.from_quat(orientation)
    rvec = r.as_rotvec()
    euler_degree = r.as_euler('xyz',degrees=True)
    cv2.putText(frame, 'EF', (10, text_height), font, 2, (0, 0, 255), 5, cv2.LINE_AA)
    cv2.putText(frame, 'Euler_degree ' + str(euler_degree), (10, text_height + 100), font, 2, (0, 0, 255), 5, cv2.LINE_AA)
    cv2.putText(frame, 'tvecs ' + str(position), (10, text_height + 200), font, 2, (0, 0, 255), 5, cv2.LINE_AA)
    cv2.drawFrameAxes(frame, cam_matrix, cam_dist, rvec, np.array([0.0, 0.0, 0.0]), 3)
    # frame = draw_EF_axis(frame, rvec, position, cam_matrix, cam_dist)

    return frame

def collect_files(path):
    for path, subdirs, files in os.walk(path):
        for name in files:
            if name == 'boards.yaml':
                board_path = os.path.join(path, name)
            elif name == 'calibration.json':
                intrinsic_path = os.path.join(path, name)

    return path, board_path, intrinsic_path

def initiate_workspace(datasetPath, intrinsicPath):
    pathO = args.PathOpts(image_path=datasetPath)
    cam = args.CameraOpts(motion_model="calibrate_board",
                          calibration=intrinsicPath)
    runt = args.RuntimeOpts()
    opt = args.OptimizerOpts(outlier_threshold=1.2, fix_intrinsic=True)
    c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
    workspace = c.execute_new()
    workspace.pose_table = make_pose_table(workspace.point_table, workspace.boards,
                                                workspace.cameras, method="solvePnP")
    return workspace

def main():
    directory = 'D:\MY_DRIVE_N\Masters_thesis\Dataset\intrinsic_26June\cam222'
    path, board_path, intrinsic_path = collect_files(directory)

    workspace = initiate_workspace(directory, intrinsic_path)
    cam_matrix, cam_dist = workspace.cameras[0].intrinsic, workspace.cameras[0].dist
    for idx,img in enumerate(workspace.names.image):
        image_path = os.path.join(path, img)
        frame = cv2.imread(image_path)
        boards = [b for b,v in enumerate(workspace.pose_table.valid[0][idx]) if v==True]
        text_height = 200
        for b in boards:
            x= workspace.pose_table.poses[0][idx][b]
            rvecs, tvecs = rtvec.split(rtvec.from_matrix(workspace.pose_table.poses[0][idx][b]))
            euler_deg = rtvec.euler_angle(rvecs)
            # frame, text_height = draw(frame, b, euler_deg, rvecs, tvecs, cam_matrix, cam_dist, text_height)

            corners, ids, _ = cv2.aruco.detectMarkers(frame[:,:,0], workspace.boards[b].board.dictionary, parameters=cv2.aruco.DetectorParameters_create())
            marker_length = workspace.boards[b].marker_length
            frame = draw_marker_pose(frame, ids, corners, marker_length, cam_matrix, cam_dist)
            frame, text_height = draw(frame, b, euler_deg, rvecs, tvecs, cam_matrix, cam_dist, text_height)
        cv2.imwrite(image_path, frame)


if __name__ == '__main__':
    main()
    # directory = 'D:\MY_DRIVE_N\Masters_thesis\Dataset\handEye_gripper/08320220/08320220/angle_check_pnp_p3p'
    # path, board_yaml, camera_intrinsic = collect_files(directory)
    #
    # camera_file = open(camera_intrinsic)
    # cameras = json.load(camera_file)
    # cam_name = [k for k,v in cameras['cameras'].items()][0]
    # cam_matrix = np.array(cameras['cameras'][cam_name]['K'])
    # cam_dist = np.array(cameras['cameras'][cam_name]['dist'])
    #
    # board_config_file = board.load_config(board_yaml)
    # base_gripper = {}
    # world_camera = {}
    # base_gripper['R'] = []
    # base_gripper['T'] = []
    # world_camera['R'] = []
    # world_camera['T'] = []
    # json_dict = {}
    # load_hande_eye = False
    # if load_hande_eye == False:
    #     for filename in os.listdir(path):
    #         print(filename)
    #         image_path = os.path.join(path, filename)
    #         frame = cv2.imread(image_path)
    #         b = boards.Boards(boards=board_yaml, detect=image_path, pixels_mm=10, show_image=False)
    #         # b = boards.Boards(boards=board_yaml, detect=detect_img)
    #         detection = b.execute()
    #         detected_boards = [idx for idx,d in enumerate(detection) if len(d.ids)>0]
    #
    #         # base_gripper_R, base_gripper_T = get_base_gripper_transformation(end_effector_poses, filename)
    #         # frame = write_base_gripper_pose(frame, end_effector_poses, filename, text_height=200)
    #         text_height = 200
    #         for idx,board_num in enumerate(detected_boards):
    #             ids = detection[board_num].ids
    #             corners = detection[board_num].corners
    #             detected_board = board_config_file[list(board_config_file)[board_num]]
    #             adjusted_points = detected_board.adjusted_points
    #             objpoints = np.array([adjusted_points[a] for a in ids], dtype='float64').reshape((-1, 3))
    #             detect_board_config = board_config(detected_board)
    #             ret, rvecs, tvecs, euler_deg, view_angle = board_pose(objpoints,
    #                                     corners, ids, detect_board_config, cam_matrix, cam_dist, frame, method="solvePnP_P3P")
    #             if ret:
    #                 rtvecs = join(rvecs, tvecs)
    #                 rtmatrix = np.linalg.inv(to_matrix(rtvecs))
    #                 R_matrix, T = matrix.split(rtmatrix)
    #                 world_camera['R'].append(R_matrix)
    #                 world_camera['T'].append(T)
    #                 # base_gripper['R'].append(base_gripper_R)
    #                 # base_gripper['T'].append(base_gripper_T)
    #                 frame, text_height = draw(frame, board_num, euler_deg, view_angle, tvecs, text_height)
    #         # frame = write_base_gripper_pose(frame, end_effector_poses, filename, text_height=text_height+400)
    #         pass
    #         cv2.imwrite(image_path, frame)
    #         pass
    #
    # pass
