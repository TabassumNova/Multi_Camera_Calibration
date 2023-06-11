import cv2
import numpy as np
from src.multical.transform.matrix import *
from src.multical.transform.rtvec import *

def error_board_param(ws, pose_init_board):
    '''
    Camera1 sees board1 and board2.
    camera1_board1 = cam1_board2 @ board2_masterBoard @ masterBoard_board1
    :param ws: workspace
    :param pose_init_board: previously calculated board parameters
    :return: error_dict
    '''
    cam = 0
    camera = ws.pose_table._index_select(0, axis=0)
    detected_points_perCam = ws.detected_points[cam]
    camera_matrix = ws.cameras[cam].intrinsic
    camera_dist = ws.cameras[cam].dist
    master_board = 0
    pose = 1
    error_dict = {}
    for img in range(0, ws.sizes.image):
        image = camera._index_select(img, axis=0)
        detected_points_perImg = detected_points_perCam[img]
        detected_boards = [idx for idx, value in enumerate(image.valid) if value == True]
        if len(detected_boards) >= 2:
            for board1 in detected_boards:
                corners = detected_points_perImg[board1].corners
                ids = detected_points_perImg[board1].ids
                objectPoints = np.array([ws.boards[board1].adjusted_points[i] for i in ids], dtype='float64').reshape(
                    (-1, 3))
                for board2 in detected_boards:
                    pose_cam_board2 = np.linalg.inv(image.poses[board2])  # cam wrto board2(base)
                    pose_board2_master = pose_init_board.poses[board2]
                    pose_master_board1 = np.linalg.inv(pose_init_board.poses[board1])
                    pose_board1_cam = np.linalg.inv(pose_cam_board2 @ pose_board2_master @ pose_master_board1)
                    rtvec = from_matrix(pose_board1_cam)
                    rvec, tvec = split(rtvec)
                    imagePoints, _ = cv2.projectPoints(objectPoints, rvec, tvec, camera_matrix, camera_dist)
                    error = imagePoints.reshape([-1, 2]) - corners
                    error_dict[pose] = error
                    pose += 1

def reprojection_error_calculation():
    # Reprojection error calculation :(
    cam = 0
    camera_matrix = self.cameras[cam].intrinsic
    camera_dist = self.cameras[cam].dist
    master_board = 0
    pose = 1
    error_dict = {}
    for img in range(0, self.sizes.image):
        table = self.pose_table._index_select(img, axis=1)
        detected_boards = [idx for idx, value in enumerate(table.valid[cam]) if value == True]
        if len(detected_boards) >= 2:
            for board1 in detected_boards:
                # l = len(self.detected_points[cam][img][board1].ids)
                # if len(self.detected_points[cam][img][board1].ids):
                corners = self.detected_points[cam][img][board1].corners
                ids = self.detected_points[cam][img][board1].ids
                objectPoints = np.array([self.boards[board1].adjusted_points[i] for i in ids], dtype='float64').reshape(
                    (-1, 3))
                for board2 in detected_boards:
                    # ### test
                    # pose_cam_board2 = np.linalg.inv(
                    #     self.pose_table.poses[cam][img][board2])  # cam wrto board2(base)
                    # pose_cam_board1 = np.linalg.inv(
                    #     self.pose_table.poses[cam][img][board1])  # cam wrto board1(base)
                    # pose_board2_board1 = matrix.relative_to(pose_cam_board2,
                    #                                         pose_cam_board1)  # board2 wrto board1(base)
                    # ## new
                    # pose_board2_cam = (self.pose_table.poses[cam][img][board2])
                    # pose_board1_cam = (self.pose_table.poses[cam][img][board1])
                    # rtvec = from_matrix((pose_board2_cam))
                    # rvec, tvec = split(rtvec)
                    # rvec2, tvec2 = self.inversePerspective(rvec, tvec.reshape((3, 1)))
                    # rtvec = from_matrix(np.linalg.inv(pose_board1_cam))
                    # rvec1, tvec1 = split(rtvec)
                    # composedRvec, composedTvec = self.relativePosition(rvec1, tvec1, rvec2, tvec2)
                    # j = join(np.squeeze(composedRvec), np.squeeze(composedTvec))
                    # pose_board1_board2 = to_matrix(j)
                    # estimate1_cam_board1 = np.linalg.inv(pose_board2_cam) @ np.linalg.inv(pose_board1_board2)
                    # err = estimate1_cam_board1 - np.linalg.inv(pose_board1_cam)
                    # rtvec = from_matrix(np.linalg.inv(estimate1_cam_board1))
                    # rvec, tvec = split(rtvec)
                    # imagePoints, _ = cv2.projectPoints(objectPoints, rvec, tvec, camera_matrix, camera_dist)
                    # error1 = imagePoints.reshape([-1, 2]) - corners
                    # ## new
                    # estimate_cam_board1 = pose_cam_board2 @ pose_board2_board1
                    # err = estimate_cam_board1 - pose_cam_board1
                    # rtvec = from_matrix(np.linalg.inv(estimate_cam_board1))
                    # rvec, tvec = split(rtvec)
                    # imagePoints, _ = cv2.projectPoints(objectPoints, rvec, tvec, camera_matrix, camera_dist)
                    # error = imagePoints.reshape([-1, 2]) - corners
                    # ### test
                    pose_cam_board2 = np.linalg.inv(self.pose_table.poses[cam][img][board2])  # cam wrto board2(base)
                    pose_board2_master = matrix.relative_to(pose_init_board.poses[board2],
                                                            pose_init_board.poses[master_board])
                    pose_master_board1 = matrix.relative_to(pose_init_board.poses[master_board],
                                                            pose_init_board.poses[board1])
                    transformation = np.linalg.inv(pose_cam_board2 @ pose_board2_master @ pose_master_board1)
                    rtvec = from_matrix(transformation)
                    rvec, tvec = split(rtvec)
                    imagePoints, _ = cv2.projectPoints(objectPoints, rvec, tvec, camera_matrix, camera_dist)
                    error = imagePoints.reshape([-1, 2]) - corners
                    error_dict[pose] = error
                    pose += 1
                    pass
    pass

def inversePerspective(self, rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    R = np.matrix(R).T
    invTvec = np.dot(R, np.matrix(-tvec))
    invRvec, _ = cv2.Rodrigues(R)
    return invRvec, invTvec

def relativePosition(self, rvec1, tvec1, rvec2, tvec2):
    """ Get relative position for rvec2 & tvec2. Compose the returned rvec & tvec to use composeRT with rvec2 & tvec2 """
    rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
    rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))
    # Inverse the second marker
    invRvec, invTvec = self.inversePerspective(rvec2, tvec2)
    info = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
    composedRvec, composedTvec = info[0], info[1]
    composedRvec = composedRvec.reshape((3, 1))
    composedTvec = composedTvec.reshape((3, 1))
    return composedRvec, composedTvec