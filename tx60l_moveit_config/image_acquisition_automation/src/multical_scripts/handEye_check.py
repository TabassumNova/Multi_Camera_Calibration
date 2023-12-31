import math

import numpy as np
from numba import cuda, jit
from src.extrinsic2pyramid.camera_pose_visualizer import CameraPoseVisualizer
import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
from src.multical.tables import *
from structs.struct import transpose_structs, invert_keys
from structs.numpy import shape_info, struct, Table, shape
from structs.struct import struct, to_dicts, transpose_lists
from scipy.optimize import least_squares
import copy
import json
import os
from src.multical.transform import common, rtvec
from src.multical_scripts.extrinsic_viz import *
import plotly.express as px
import pandas as pd
from src.multical_scripts.extrinsic_viz_board import *
from sklearn.cluster import MeanShift

base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V35"

class handEye1():
    def __init__(self, base_path, master_cam=0, master_board=0):
        self.base_path = base_path
        self.master_cam = master_cam
        self.master_board = master_board
        self.datasetPath = base_path
        self.boardPath = None
        self.poseJsonPath = None
        self.intrinsicPath = None
        self.workspace = None
        self.handEyeGripper = None
        self.gripper_pose = {}
        self.collect_files()
        self.board_pose = {}
        self.board_pose_temp = []
        self.cam_pose = None
        # self.cam_pose2 = {}
        # self.board_pose2 = {}
        self.camera_color = {}
        self.all_handEye = {}
        self.board_param = []
        self.campose_param = {}
        self.camintrinsic_param = []
        self.inliers = None
        self.final_campose = {}
        self.calib_obj = {}
        self.calibObj_mean = {}

    def collect_files(self):
        """
        It takes folder containing only one camera, board.yaml, calibration.json, gripper_pose.json
        :return:
        """
        for path, subdirs, files in os.walk(self.base_path):
            for name in files:
                if name == 'boards.yaml':
                    self.boardPath = os.path.join(self.base_path, name)
                elif name == 'calibration.json':
                    self.intrinsicPath = os.path.join(self.base_path, name)
                elif name == 'gripper_pose.json':
                    self.poseJsonPath = os.path.join(self.base_path, name)
                elif name == "handEyeGripper.json":
                    self.handEyeGripper = os.path.join(self.base_path, name)
                self.handEyeGripper = os.path.join(self.base_path, "handEyeGripper.json")

    def export_workspace(self):
        workspace_pickle = os.path.join(self.base_path, "workspace.pkl")
        with open(workspace_pickle, "wb") as file:
            pickle.dump(self.workspace, file)

    def camera_intrinsic_dataset(self):
        intrinsic_dataset = {}

        for cam_id, cam in enumerate(self.workspace.names.camera):
            intrinsic_dataset[cam] = {}
            board_list = []
            image_list = []
            for idx, board_id in enumerate(self.workspace.cameras[cam_id].intrinsic_dataset['board_ids']):
                board_list.append(self.workspace.names.board[int(board_id)])
                image_id = self.workspace.cameras[cam_id].intrinsic_dataset['image_ids'][idx]
                image_list.append(self.workspace.names.image[int(image_id)])
                pass
            intrinsic_dataset[cam]['boards'] = board_list
            intrinsic_dataset[cam]['images'] = image_list

        json_object = json.dumps(intrinsic_dataset, indent=4)
        # Writing to sample.json
        json_path = os.path.join(self.base_path, "intrinsic_dataset.json")
        with open(json_path, "w") as outfile:
            outfile.write(json_object)
        pass

    def initiate_workspace(self, show_all_poses=False):
        pathO = args.PathOpts(image_path=self.datasetPath)
        cam = args.CameraOpts(calibration=self.intrinsicPath, intrinsic_error_limit=0.5)
        # pose_estimation_method = "solvePnPRansac"
        pose_estimation_method = "solvePnPGeneric"
        runt = args.RuntimeOpts(pose_estimation=pose_estimation_method, show_all_poses=True)
        opt = args.OptimizerOpts(outlier_threshold=1.2, fix_intrinsic=True, adjust_outliers=False)
        c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
        self.workspace = c.execute_board()
        self.export_workspace()
        if not self.intrinsicPath:
            self.camera_intrinsic_dataset()

        for c in range(len(self.workspace.names.camera)):
            self.camintrinsic_param.extend(self.workspace.cameras[c].param_vec.tolist())

        # self.refine_intrinsic()
        self.board_pose = self.workspace.board_poses
        for b in range(len(self.workspace.names.board)):
            t = rtvec.from_matrix(self.workspace.board_poses.poses[b])
            self.board_param.extend(rtvec.from_matrix(self.workspace.board_poses.poses[b]).tolist())
        pass
        # self.refine_detected_points()
        self.set_Cam_color()

    def refine_detected_points(self):
        for idxc, cam in enumerate(self.workspace.names.camera):
            camera = self.workspace.cameras[idxc]
            for idxi, img in enumerate(self.workspace.names.image):
                for idxb, board in enumerate(self.workspace.names.board):
                    board = self.workspace.boards[idxb]
                    detections = self.workspace.detected_points[idxc][idxi][idxb]
                    if board.has_min_detections(detections):
                        undistorted = camera.undistort_points(detections.corners).astype('float32')
                        objPoints = board.points[detections.ids].astype('float32')
                        valid, rvec, tvec, inliers0 = cv2.solvePnPRansac(objPoints, undistorted, camera.intrinsic,
                                                                        camera.dist, reprojectionError=0.4)

                        point_table_valid = self.workspace.point_table.valid[idxc][idxi][idxb]
                        inliers = np.zeros((point_table_valid.shape), dtype=bool)

                        if valid:
                            inliers1 = inliers0.reshape((1, -1))[0].tolist()
                            x = [detections.ids[i] for i in inliers1]
                            inliers[x] = True
                        self.workspace.point_table.valid[idxc][idxi][idxb] = np.array(inliers)
        pass



    def refine_intrinsic(self):
        def decode_intrinsic(intrinsic_param):
            dist = intrinsic_param[5:10]
            fx, fy, x0, y0, s = intrinsic_param[0], intrinsic_param[1], intrinsic_param[2], intrinsic_param[3], intrinsic_param[4]
            intrinsic = [
                [fx, s, x0],
                [0, fy, y0],
                [0, 0, 1],
            ]
            return np.array(intrinsic), np.array(dist)
        def fun(param_vec, cam_idx):
            error_list = []
            camera = self.workspace.cameras[cam_idx]
            cam_matrix, cam_dist = decode_intrinsic(param_vec[0: 10])
            for idxi, image in enumerate(self.workspace.names.image):
                for idxb, board in enumerate(self.workspace.names.board):
                    board = self.workspace.boards[idxb]
                    detections = self.workspace.detected_points[idxc][idxi][idxb]
                    if board.has_min_detections(detections):
                        undistorted = camera.undistort_points(detections.corners).astype('float32')
                        objPoints = board.points[detections.ids].astype('float32')
                        valid, rvec, tvec, inliers = cv2.solvePnPRansac(objPoints, undistorted, cam_matrix,
                                                                            cam_dist, reprojectionError=1.0)
                        if valid == True and len(inliers.tolist())>6:
                            objPoints2 = np.array([objPoints[i[0]] for i in inliers])
                            cornerPoints2 = np.array([detections.corners[i[0]] for i in inliers])
                            undistorted2 = np.array([undistorted[i[0]] for i in inliers])
                            valid2, rvec2, tvec2, error2 = cv2.solvePnPGeneric(objPoints2, undistorted2, cam_matrix,
                                                                           cam_dist)
                            imP, _ = cv2.projectPoints(np.copy(objPoints), np.array(rvec2), np.array(tvec2), cam_matrix, cam_dist)
                            error = np.linalg.norm(detections.corners - imP.reshape([-1, 2]), axis=-1)
                            # if np.isfinite(error.all()):
                            #     error_list.extend(list(error))
                            for idx, e in enumerate(error):
                                if e > 1:
                                    error[idx] = 10
                        else:
                            error = np.ones((detections.ids.shape))+10
                        error_list.extend(list(error))
            return error_list

        def evaluation(param_vec, inliers, cam_idx):
            error0 = fun(param_vec, cam_idx)
            error = np.array([error0[i] for i, v in enumerate(inliers) if v == True])
            return error


        for idxc, cam in enumerate(self.workspace.names.camera):
            param_vec = self.workspace.cameras[idxc].param_vec.tolist()
            point_errors = fun(param_vec, idxc)
            inliers = np.ones(len(point_errors))
            for idx, e in enumerate(point_errors):
                if e>10:
                    inliers[idx] = 0
            x = 0

            for i in range(1):
                info(f"Adjust_outliers {i}:")
                res = least_squares(evaluation, param_vec,
                                    verbose=2, x_scale='jac', f_scale=1.0, ftol=1e-4, max_nfev=100,
                                    method='trf', loss='linear', args=(inliers, idxc))
                point_errors = fun(res.x, idxc)
                threshold = np.quantile(point_errors, 0.75)
                inliers = np.array(point_errors < threshold).flatten()
            point_errors = fun(res.x, idxc)

    def handEye_gripper(self, camera, board):
        board_cam_pose = self.workspace.pose_table._index_select(camera, axis=0)._index_select(board, axis=1)
        objPoints = self.workspace.boards[board].adjusted_points

        R_cam2world_list = []
        t_cam2world_list = []
        R_base2gripper_list = []
        t_base2gripper_list = []
        corner_list = []
        point3D_list = []
        image_name = self.workspace.names.image
        image_list = []
        for img in image_name:
            p = image_name.index(img)
            grip_pose = img[:-4][1:]
            camBoard_valid = board_cam_pose.valid[p]
            if camBoard_valid:
                board_to_cam = board_cam_pose.poses[p]
                R_cam2world, t_cam2world = matrix.split(np.linalg.inv(board_to_cam))
                R_base2gripper, t_base2gripper = matrix.split(np.linalg.inv(self.gripper_pose[grip_pose]))
                R_cam2world_list.append(R_cam2world)
                t_cam2world_list.append(t_cam2world)
                R_base2gripper_list.append(R_base2gripper)
                t_base2gripper_list.append(t_base2gripper)
                image_list.append(img)
                corners = self.workspace.detected_points[0][p][board].corners
                ids = self.workspace.detected_points[0][p][board].ids
                point3D = [objPoints[x] for x in ids]
                corner_list.append(corners)
                point3D_list.append(point3D)
        base_wrt_cam, gripper_wrt_world, camera_wrt_world, base_wrt_gripper, estimated_gripper_base, error1, error2 = self.hand_eye_robot_world(np.array(R_cam2world_list),
                                            np.array(t_cam2world_list), np.array(R_base2gripper_list), np.array(t_base2gripper_list))

        handEye_struct = struct(camera=self.workspace.names.camera[camera], board=self.workspace.names.board[board], base_wrt_cam=base_wrt_cam, gripper_wrt_world=gripper_wrt_world,
                                camera_wrt_world=camera_wrt_world, base_wrt_gripper=base_wrt_gripper, estimated_gripper_base=estimated_gripper_base, images=image_list,
                                corners=corner_list, objPoints=point3D_list)

        # self.reprojectionError_Calculation_new(handEye_struct)
        # self.test_robotMove(handEye_struct)

        return base_wrt_cam, gripper_wrt_world, camera_wrt_world, base_wrt_gripper, image_list

    def viewPlanSimple(self, limit_board_image=10):
        '''
        If the number of hand-eye transformations is very few. with cube
        '''
        num_cameras = len(self.workspace.names.camera)
        num_boards = len(self.workspace.names.board)
        handEyeGripper_dict = {}
        for cam_id, cam in enumerate(self.workspace.names.camera):
            if cam not in handEyeGripper_dict:
                handEyeGripper_dict[cam] = {}
            viewed_boards = [b for b in range(0, num_boards) if
                             self.workspace.pose_table.valid[cam_id][:, b].sum() > limit_board_image]
            for board_id in viewed_boards:
                board = self.workspace.names.board[board_id]
                if board not in handEyeGripper_dict[cam]:
                    handEyeGripper_dict[cam][board] = {}
                base_wrt_cam, gripper_wrt_world, camera_wrt_world, base_wrt_gripper, image_list = self.handEye_gripper(cam_id, board_id)
                handEyeGripper_dict[cam][board]['base_wrt_cam'] = base_wrt_cam.tolist()
                handEyeGripper_dict[cam][board]['gripper_wrt_world'] = gripper_wrt_world.tolist()
                handEyeGripper_dict[cam][board]['camera_wrt_world'] = camera_wrt_world.tolist()
                handEyeGripper_dict[cam][board]['base_wrt_gripper'] = base_wrt_gripper.tolist()
                handEyeGripper_dict[cam][board]['image_list'] = image_list.tolist()

        json_object = json.dumps(handEyeGripper_dict, indent=4)
        # Writing to sample.json
        path = os.path.join(self.base_path, 'handEyeGripper.json')
        with open(path, "w") as outfile:
            outfile.write(json_object)
        pass

    def viewPlanRobust(self, limit_board_image=10):
        '''
        This function will return handEye pose for Camera-Base and Board-Gripper
        It will work if the hand-eye has robust transformation with isohedron
        '''

        num_cameras = len(self.workspace.names.camera)
        num_boards = len(self.workspace.names.board)

        base_camera_list = {}
        gripper_board_list = {}
        serial = 0
        cam_visualizer = CameraPoseVisualizer([-2000, 2000], [-2000, 2000], [-2000, 2000])
        cam_layout = go.Figure()
        cam_layout.add_annotation(dict(font=dict(color='black', size=20), x=0, y=0.12, showarrow=False, text='camera',
                                         textangle=0, xanchor='left', xref="paper", yref="paper"))
        board_visualizer = CameraPoseVisualizer([-2000, 2000], [-2000, 2000], [-2000, 2000])
        board_layout = go.Figure()
        board_layout.add_annotation(dict(font=dict(color='black', size=20), x=0, y=0.12, showarrow=False, text='board',
                                       textangle=0, xanchor='left', xref="paper", yref="paper"))
        for cam_id, cam in enumerate(self.workspace.names.camera):
            if cam not in base_camera_list:
                base_camera_list[cam] = {}
                base_camera_list[cam]['group'] = []
            viewed_boards = [b for b in range(0, num_boards) if
                             self.workspace.pose_table.valid[cam_id][:, b].sum() > limit_board_image]
            for board_id, board in enumerate(self.workspace.names.board):
                if board not in gripper_board_list:
                    gripper_board_list[board] = {}
                    gripper_board_list[board]['group'] = []
                if board_id in viewed_boards:
                    base_wrt_cam, gripper_wrt_world = self.handEye_gripper(cam_id, board_id)
                    base_camera_list[cam]['group'].append(rtvec.from_matrix(base_wrt_cam))
                    gripper_board_list[board]['group'].append(rtvec.from_matrix(gripper_wrt_world))
                    data_cam = cam_visualizer.extrinsic2pyramid(base_wrt_cam, color=self.camera_color[cam],
                                                        focal_len_scaled=0.1, aspect_ratio=0.3, show_legend=False,
                                                        hover_template=cam)
                    data_board = board_visualizer.extrinsic2pyramid(gripper_wrt_world, color='red',
                                                                focal_len_scaled=0.1, aspect_ratio=0.3,
                                                                show_legend=False,
                                                                hover_template=board)
                    cam_layout.add_trace(data_cam)
                    board_layout.add_trace(data_board)

        for cam_id, cam in enumerate(self.workspace.names.camera):
            if len(base_camera_list[cam]['group'])>=3:
                mean = rtvec.to_matrix(common.mean_robust(np.array(base_camera_list[cam]['group'])))
                base_camera_list[cam]['mean'] = mean
            else:
                base_camera_list[cam]['mean'] = rtvec.to_matrix(base_camera_list[cam]['group'][0])
            data_cam_mean = cam_visualizer.extrinsic2pyramid(base_camera_list[cam]['mean'], color='black',
                                                        focal_len_scaled=0.1, aspect_ratio=0.3, show_legend=False,
                                                        hover_template=cam)
            cam_layout.add_trace(data_cam_mean)
        for board_id, board in enumerate(self.workspace.names.board):
            if len(gripper_board_list[board]['group']) >= 3:
                mean = rtvec.to_matrix(common.mean_robust(np.array(gripper_board_list[board]['group'])))
                gripper_board_list[board]['mean'] = mean
            else:
                gripper_board_list[board]['mean'] = rtvec.to_matrix(gripper_board_list[board]['group'][0])
            data_board_mean = board_visualizer.extrinsic2pyramid(gripper_board_list[board]['mean'], color='black',
                                                            focal_len_scaled=0.1, aspect_ratio=0.3,
                                                            show_legend=False,
                                                            hover_template=board)

            board_layout.add_trace(data_board_mean)
        cam_layout.show()
        board_layout.show()
        pass

    def set_Cam_color(self):
        colors = ['red', 'green', 'blue', 'cyan', 'magenta', 'lime', 'pink', 'teal', 'darkcyan', 'violet', 'brown', 'indigo']
        for idx, cam in enumerate(self.workspace.names.camera):
            self.camera_color[cam] = colors[idx]

    @staticmethod
    def hand_eye_robot_world(cam_world_R, cam_world_t, base_gripper_R, base_gripper_t):
        base_cam_r, base_cam_t, gripper_world_r, gripper_world_t = \
            cv2.calibrateRobotWorldHandEye(cam_world_R, cam_world_t, base_gripper_R, base_gripper_t, method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH)

        base_wrt_cam = matrix.join(base_cam_r, base_cam_t.reshape(-1))
        gripper_wrt_world = matrix.join(gripper_world_r, gripper_world_t.reshape(-1))
        camera_wrt_world = matrix.join(cam_world_R, cam_world_t)
        base_wrt_gripper = matrix.join(base_gripper_R, base_gripper_t)

        err = matrix.transform(base_wrt_cam, camera_wrt_world) - matrix.transform(base_wrt_gripper, gripper_wrt_world)
        ZB = matrix.transform(base_wrt_cam, camera_wrt_world)
        error2 = base_wrt_gripper - matrix.transform(ZB, np.linalg.inv(gripper_wrt_world))
        estimated_gripper_base = np.linalg.inv(matrix.transform(ZB, np.linalg.inv(gripper_wrt_world)))

        return base_wrt_cam, gripper_wrt_world, camera_wrt_world, base_wrt_gripper, estimated_gripper_base, err, error2

    def reprojectionError_Calculation_new(self, handEye_struct):
        error_dict = {}
        reprojection_error_dict = {}
        for pose in range(0, len(handEye_struct.images)):
            # t1 = matrix.transform(handEye_struct.gripper_wrt_world, handEye_struct.base_wrt_gripper[pose])
            # transform = matrix.transform(t1, np.linalg.inv(handEye_struct.base_wrt_cam))
            t1 = matrix.transform(np.linalg.inv(handEye_struct.base_wrt_cam), handEye_struct.base_wrt_gripper[pose])
            transform = matrix.transform(t1, (handEye_struct.gripper_wrt_world))
            error_dict[pose] = handEye_struct.camera_wrt_world[pose] - transform
            camera_matrix, camera_dist = self.workspace.cameras[0].intrinsic, self.workspace.cameras[0].dist
            rvec, tvec = rtvec.split(rtvec.from_matrix(np.linalg.inv(transform)))
            imagePoints, _ = cv2.projectPoints(np.array(handEye_struct.objPoints[pose]),
                                               rvec, tvec, camera_matrix, camera_dist)
            reprojection_error_dict[pose] = imagePoints.reshape([-1, 2]) - handEye_struct.corners[pose]
        pass

    def test_robotMove(self, handEye_struct):
        estimated_gripper_base_list = []
        p1 = handEye_struct.camera_wrt_world[0]
        cam_gripper1 = rtvec.from_matrix(matrix.transform(p1, np.linalg.inv(handEye_struct.gripper_wrt_world)))
        angle1 = rtvec.euler_angle(cam_gripper1[:3])
        base_gripper1 = handEye_struct.base_wrt_gripper[0]
        image1 = handEye_struct.images[0]
        for idx2, p2 in enumerate(handEye_struct.camera_wrt_world):
            image2 = handEye_struct.images[idx2]
            cam_gripper2 = rtvec.from_matrix(matrix.transform(p2, np.linalg.inv(handEye_struct.gripper_wrt_world)))
            angle2 = rtvec.euler_angle(cam_gripper2[:3])
            diff = cam_gripper1 - cam_gripper2
            angle_diff = rtvec.euler_angle(diff[:3])
            base_gripper2 = handEye_struct.base_wrt_gripper[idx2]
            cam_gripper_diff = rtvec.to_matrix(diff)
            # new
            # estimated_base_gripper = matrix.transform((base_gripper1), (np.linalg.inv(cam_gripper_diff)))
            # error = base_gripper2 - (estimated_base_gripper)
            # estimated_gripper_base_list.append(np.linalg.inv(estimated_base_gripper))
            # new
            estimated_base_gripper = rtvec.from_matrix(base_gripper1) + rtvec.from_matrix(np.linalg.inv(cam_gripper_diff))
            error = base_gripper2 - rtvec.to_matrix(estimated_base_gripper)
            estimated_gripper_base_list.append(np.linalg.inv(rtvec.to_matrix(estimated_base_gripper)))
        return estimated_gripper_base_list


    def set_gripper_pose(self):
        file = open(self.poseJsonPath)
        data = json.load(file)
        num_pose = self.workspace.sizes.image

        image_name = self.workspace.names.image
        for img in image_name:
            i = img[:-4][1:]
            if i in data:
                position = [float(j) for j in data[str(i)]["position (x,y,z)"]]
                orientation = [float(j) for j in data[str(i)]["orintation (w,x,y,z)"]]
                r = R.from_quat([orientation[1], orientation[2], orientation[3], orientation[0]])
                rvec = r.as_rotvec()
                tvec = np.array(position)
                rt_matrix = rtvec.to_matrix(rtvec.join(rvec, tvec))
                self.gripper_pose[i] = rt_matrix
        pass

    def export_handEyeGripper(self, handEye_struct):
        json_dict = {}
        json_dict['camera'] = handEye_struct.camera
        json_dict['board'] = handEye_struct.board
        json_dict["base_wrt_cam"] = handEye_struct.base_wrt_cam.tolist()
        json_dict["gripper_wrt_world"] = handEye_struct.gripper_wrt_world.tolist()
        json_object = json.dumps(json_dict, indent=4)
        # Writing to sample.json
        with open(self.handEyeGripper, "w") as outfile:
            outfile.write(json_object)
        pass

    def estimate_camera_board_poses(self):
        '''
        for Master_cam to Slave_cam
        :return:
        '''
        num_cameras = len(self.workspace.names.camera)
        handEye_dict = {}
        for cam in range(0, num_cameras):
            handEye_dict[cam] = self.handEye_table(master_cam=cam)

    # @jit(target_backend='cuda')
    def handEye_table(self, master_cam=0, limit_image=2, num_adjustments=5, limit_board_image=2, calculate_handeye=True, check_cluster=True):
        '''
        handEye for Master_cam to Slave_cam
        :param limit_image: number of image allowed for hand eye pair
               limit_board_image : number of image allowed to choose as major board seen by cameras
        :return:
        '''
        num_cameras = len(self.workspace.names.camera)
        num_boards = len(self.workspace.names.board)

        handEye_dict = {}
        serial = 0
        master_boards = [b for b in range(0, num_boards) if self.workspace.pose_table.valid[master_cam][:, b].sum() > limit_board_image]
        for slave_cam in range(0, num_cameras):
            if slave_cam != master_cam:
                slave_boards = [b for b in range(0, num_boards) if self.workspace.pose_table.valid[slave_cam][:, b].sum() > limit_board_image]

                for boardM in master_boards:
                    # if boardM not in self.board_pose:
                    #     self.board_pose[boardM] = {}

                    for boardS in slave_boards:
                        masterR, masterT, slaveR, slaveT, image_list = self.master_slave_pose(master_cam, boardM,
                                                                                         slave_cam, boardS, check_cluster)

                        if len(image_list)>limit_image:
                            # board_wrt_boardM, slave_wrt_master, world_wrt_camera, \
                            # base_wrt_gripper, err, err2 = hand_eye.hand_eye_robot_world(masterR, masterT, slaveR, slaveT)
                            print("Group: ",serial,"MasterCam: ",master_cam, " MasterB: ", boardM, " SlaveCam: ", slave_cam, " SlaveB: ", boardS, " images: ", image_list)
                            if calculate_handeye:
                                slaveCam_wrt_masterCam, slaveB_wrt_masterB, masterCam_wrt_masterB, slaveCam_wrt_slaveB, \
                                estimated_slaveB_slaveCam, err, err2 = self.hand_eye_robot_world(masterR, masterT, slaveR, slaveT)
                            else:
                                slaveCam_wrt_masterCam, slaveB_wrt_masterB, masterCam_wrt_masterB, slaveCam_wrt_slaveB, \
                                estimated_slaveB_slaveCam, err, err2 = np.eye(4), np.eye(4), np.tile(np.eye(4), (len(image_list),1,1)),\
                                                            np.tile(np.eye(4), (len(image_list),1,1)), \
                                                                       np.tile(np.eye(4), (len(image_list),1,1)), 0, 0

                                ## for board pose
                            if boardM not in self.calib_obj:
                                self.calib_obj[boardM] = {}
                            if boardS not in self.calib_obj[boardM]:
                                self.calib_obj[boardM][boardS] = []
                                self.calib_obj[boardM][boardS].append(rtvec.from_matrix(slaveB_wrt_masterB))
                            else:
                                self.calib_obj[boardM][boardS].append(rtvec.from_matrix(slaveB_wrt_masterB))
                            if boardS in self.calib_obj and boardM in self.calib_obj[boardS]:
                                poses = [rtvec.from_matrix(np.linalg.inv(rtvec.to_matrix(p))) for p in self.calib_obj[boardS][boardM]]
                                self.calib_obj[boardM][boardS] += poses
                            ##

                            _, initial_error = self.reprojectionError_calculation(slave_cam, boardS, estimated_slaveB_slaveCam, image_list)
                            # error = sum(reprojection_error)/len(reprojection_error)

                            print("initial reprojection error: ", initial_error)
                            masterBoard_pose, slaveBoard_pose, masterBoard_angle, slaveBoard_angle, masterBoard_error, slaveBoard_error = self.collect_pose_angles(master_cam, boardM, slave_cam, boardS, image_list)
                            final_error = self.optimization(slave_cam, boardS, image_list,
                                                        slaveCam_wrt_masterCam, slaveB_wrt_masterB,
                                                        masterCam_wrt_masterB, slaveCam_wrt_slaveB, num_adjustments=num_adjustments)
                            handEye_dict[serial] = to_dicts(struct(master_cam=self.workspace.names.camera[master_cam],
                                                        master_board=self.workspace.names.board[boardM],
                                                        slave_cam = self.workspace.names.camera[slave_cam],
                                                        slave_board=self.workspace.names.board[boardS],
                                                        masterCam_wrt_masterB=masterCam_wrt_masterB.tolist(),
                                                        boardS_wrto_boardM=slaveB_wrt_masterB.tolist(),
                                                        slaveCam_wrto_masterCam=slaveCam_wrt_masterCam.tolist(),
                                                        slaveCam_wrto_slaveB = slaveCam_wrt_slaveB.tolist(),
                                                        estimated_slaveB_slaveCam = estimated_slaveB_slaveCam.tolist(),
                                                        initial_reprojection_error=str(initial_error), final_reprojection_error=str(final_error),
                                                        image_list=image_list, masterBoard_pose=masterBoard_pose,
                                                        slaveBoard_pose=slaveBoard_pose, masterBoard_angle=masterBoard_angle,
                                                        slaveBoard_angle=slaveBoard_angle, masterBoard_error=masterBoard_error,
                                                        slaveBoard_error=slaveBoard_error))
                            serial += 1



        return handEye_dict

    def collect_pose_angles(self, master_cam, boardM, slave_cam, boardS, image_list):
        masterBoard_pose = {}
        slaveBoard_pose = {}
        masterBoard_angle = {}
        slaveBoard_angle = {}
        masterBoard_error = {}
        slaveBoard_error = {}
        for img in image_list:
            img_idx = self.workspace.names.image.index(img)
            masterBoard_pose[img] = self.workspace.pose_table.poses[master_cam][img_idx][boardM].tolist()
            slaveBoard_pose[img] = self.workspace.pose_table.poses[slave_cam][img_idx][boardS].tolist()
            masterBoard_angle[img] = self.workspace.pose_table.view_angles[master_cam][img_idx][boardM].tolist()
            slaveBoard_angle[img] = self.workspace.pose_table.view_angles[slave_cam][img_idx][boardS].tolist()
            masterBoard_error[img] = self.workspace.pose_table.reprojection_error[master_cam][img_idx][boardM]
            slaveBoard_error[img] = self.workspace.pose_table.reprojection_error[slave_cam][img_idx][boardS]
        return masterBoard_pose, slaveBoard_pose, masterBoard_angle, slaveBoard_angle, masterBoard_error, slaveBoard_error

    def export_mean_cameras(self):
        filename = os.path.join(self.base_path, "meanCameras.json")
        json_object = json.dumps((self.cam_pose), indent=4)
        # Writing to sample.json
        with open(filename, "w") as outfile:
            outfile.write(json_object)
        pass

    def density_map(self, mean_calculation):
        mean_group_dict = {}
        for cam1, cam_value1 in mean_calculation.items():
            mean_group_dict[cam1] = {}
            mean_group_dict[cam1][cam1] = {}
            mean_group_dict[cam1][cam1]['extrinsic'] = np.eye(4).tolist()
            mean_group_dict[cam1][cam1]['group'] = [0]
            for cam2, cam_value2 in cam_value1.items():
                x = []
                y = []
                z = []
                group_name = []
                if cam2 != cam1:
                    if len(cam_value2['group']) > 3:
                        for idx, group_id in enumerate(cam_value2['group']):
                            tvec = cam_value2['extrinsic'][idx][3:]
                            x.append(tvec[0])
                            y.append(tvec[1])
                            z.append(tvec[2])
                            group_name.append(group_id)
                        xyz = np.vstack([x, y, z])
                        print(xyz)
                        kde = stats.gaussian_kde(xyz)
                        density = kde(xyz)
                        max_idx = np.argmax(density)
                        max_group = group_name[max_idx]
                        mean_group_dict[cam1][cam2] = {}
                        mean_group_dict[cam1][cam2]['extrinsic'] = rtvec.to_matrix(cam_value2['extrinsic'][max_idx]).tolist()
                        mean_group_dict[cam1][cam2]['group'] = [max_group]
                        mean_group_dict[cam1][cam2]['boards'] = [[self.all_handEye[cam1][max_group]['master_board'],
                                                                              self.all_handEye[cam1][max_group]['slave_board']]]
                    else:
                        mean_group_dict[cam1][cam2] = {}
                        mean_group_dict[cam1][cam2]['group'] = cam_value2['group']
                        mean_group_dict[cam1][cam2]['extrinsic'] = rtvec.to_matrix(cam_value2['extrinsic'][0]).tolist()
                        mean_group_dict[cam1][cam2]['boards'] = []
                        for g in cam_value2['group']:
                            mean_group_dict[cam1][cam2]['boards'].append([self.all_handEye[cam1][g]['master_board'],
                                                                          self.all_handEye[cam1][g]['slave_board']])
        return mean_group_dict

    def show_cluster_mean(self, mean_calculation):
        mean_group_dict = {}
        for cam1, cam_value1 in mean_calculation.items():
            mean_group_dict[cam1] = {}
            mean_group_dict[cam1][cam1] = {}
            mean_group_dict[cam1][cam1]['extrinsic'] = np.eye(4).tolist()
            mean_group_dict[cam1][cam1]['group'] = [0]
            for cam2, cam_value2 in cam_value1.items():
                if cam2 != cam1:
                    if len(cam_value2['group'])>3:
                        g = common.cluster(cam_value2['extrinsic'])
                        group = np.array(cam_value2['group'])[common.cluster(cam_value2['extrinsic'])]
                        x = rtvec.to_matrix(common.mean_robust(cam_value2['extrinsic']))
                        mean_group_dict[cam1][cam2] = {}
                        mean_group_dict[cam1][cam2]['extrinsic'] = x.tolist()
                        mean_group_dict[cam1][cam2]['group'] = group.tolist()
                        mean_group_dict[cam1][cam2]['boards'] = []
                        for g in group:
                            mean_group_dict[cam1][cam2]['boards'].append([self.all_handEye[cam1][g]['master_board'],
                                                                          self.all_handEye[cam1][g]['slave_board']])
                    else:
                        mean_group_dict[cam1][cam2] = {}
                        mean_group_dict[cam1][cam2]['group'] = cam_value2['group']
                        mean_group_dict[cam1][cam2]['extrinsic'] = rtvec.to_matrix(cam_value2['extrinsic'][0])
                        mean_group_dict[cam1][cam2]['boards'] = []
                        for g in cam_value2['group']:
                            mean_group_dict[cam1][cam2]['boards'].append([self.all_handEye[cam1][g]['master_board'],
                                                                          self.all_handEye[cam1][g]['slave_board']])

        return mean_group_dict

    def calibObj_cluster_mean(self, board=0, limit_pose=1000, mean_calculation='handEye'):
        '''
        This function is for calculating board params
        '''

        master_board = self.workspace.names.board[board]
        if mean_calculation == "multical":
            for board_id, board_name in enumerate(self.workspace.names.board):
                self.calibObj_mean[board_name] = {}
                self.calibObj_mean[board_name]['mean'] = self.board_pose.poses[board_id]
                self.calibObj_mean[board_name]['group'] = [np.eye(4)]
        if mean_calculation == "handEye":
            for board_id in self.calib_obj[board].keys():
                b = self.workspace.names.board[board_id]
                a0 = np.array(self.calib_obj[board][board_id])
                if len(a0)>limit_pose:
                    selectedG1 = np.random.choice(np.arange(len(a0)), size=limit_pose)
                    a = np.array([a0[g] for g in selectedG1])
                else:
                    a = a0
                if a.size != 0 and len(a)>3:
                    g = common.cluster(a)
                    group = a[g]
                    x = rtvec.to_matrix(common.mean_robust(group))
                    self.calibObj_mean[b] = {}
                    self.calibObj_mean[b]['mean'] = x
                    if len(group)>100:
                        selectedG = np.random.choice(np.arange(len(group)), size=100)
                    else:
                        selectedG = np.arange(len(group))
                    self.calibObj_mean[b]['group'] = [rtvec.to_matrix(group[g]) for g in selectedG]
                else:
                    self.calibObj_mean[b] = {}
                    self.calibObj_mean[b]['mean'] = rtvec.to_matrix(a[0])
                    self.calibObj_mean[b]['group'] = [rtvec.to_matrix(a[idx]) for idx,g in enumerate(a)]

        # For drawing boards
        b = Interactive_Extrinsic_Board(self.calibObj_mean, self.workspace.names.board)
        pass

    def calc_camPose_param(self, limit_images, num_adjustments, limit_board_image, calculate_handeye, check_cluster):
        cameras = self.workspace.names.camera
        # cameras = ['08320217']
        for idx, master_cam in enumerate(cameras):
            handEye_dict = self.handEye_table(master_cam=idx, limit_image=limit_images,
                                              num_adjustments=num_adjustments, limit_board_image=limit_board_image,
                                              calculate_handeye=calculate_handeye, check_cluster=check_cluster)
            self.all_handEye[master_cam] = handEye_dict

        self.calibObj_cluster_mean(limit_pose=100, mean_calculation="multical")

        mean_calculation = {}
        for cam_name, cam_group in self.all_handEye.items():
            mean_calculation[cam_name] = {}
            for group, value in cam_group.items():
                master_cam = value['master_cam']
                slave_cam = value['slave_cam']
                master_extrinsic = np.eye(4)
                slave_extrinsic = np.array(value['slaveCam_wrto_masterCam'])

                # meanGroup = "M" + master_cam + '_S' + slave_cam
                if slave_cam not in mean_calculation[cam_name]:
                    mean_calculation[cam_name][slave_cam] = {}
                    mean_calculation[cam_name][slave_cam]['extrinsic'] = from_matrix(slave_extrinsic).reshape((1, -1))
                    mean_calculation[cam_name][slave_cam]['group'] = [group]
                else:
                    mean_calculation[cam_name][slave_cam]['group'].extend([group])
                    mean_calculation[cam_name][slave_cam]['extrinsic'] = np.concatenate((mean_calculation[cam_name][slave_cam]['extrinsic'],
                                                                            from_matrix(slave_extrinsic).reshape(
                                                                                (1, -1))), axis=0)
        # for k-means cluster
        # self.cam_pose = self.show_cluster_mean(mean_calculation)
        # for density mapping
        self.cam_pose = self.density_map(mean_calculation)
        self.export_mean_cameras()
        for master, master_value in self.cam_pose.items():
            self.campose_param[master] = []
            for slave in self.workspace.names.camera:
                if slave in master_value:
                    self.campose_param[master].extend(rtvec.from_matrix(np.array(master_value[slave]['extrinsic'])).tolist())

        self.setCamBoardpose_workspace()
        pass

    def setCamBoardpose_workspace(self):
        ## for camera_poses
        # master_cam = self.workspace.names.camera[self.master_cam]  # self.master_cam = 0
        old_data = self.workspace.export_Data
        cam_pose = {}
        for cam0 in self.workspace.names.camera:
            master_cam = cam0
            cam_pose = {}
            for cam in self.workspace.names.camera:
                if cam == master_cam:
                    name = master_cam
                else:
                    name = cam + '_to_' + master_cam
                cam_pose[name] = {}
                if cam in self.cam_pose[master_cam]:
                    p = np.linalg.inv(self.cam_pose[master_cam][cam]['extrinsic'])
                    R = matrix.rotation(p).tolist()
                    T = matrix.translation(p).tolist()
                    cam_pose[name]['R'] = R
                    cam_pose[name]['T'] = T

            ## for board_poses
            # master_board = self.workspace.names.board[self.master_board]
            # board_pose = {}
            # for board in self.workspace.names.board:
            #     if board == master_board:
            #         name = master_board
            #     else:
            #         name = board + '_to_' + master_board
            #     board_pose[name] = {}
            #     if board in self.calibObj_mean:
            #         p = np.array(self.calibObj_mean[board]['mean'])
            #         R = matrix.rotation(p).tolist()
            #         T = matrix.translation(p).tolist()
            #         board_pose[name]['R'] = R
            #         board_pose[name]['T'] = T
            ## for board_poses

            ## for export
            new_data = struct(cameras=old_data.cameras, camera_poses=cam_pose)
            if self.workspace.names.camera.index(master_cam) == self.master_cam:
                filename = os.path.join(self.base_path, "Calibration_handeye.json")
                with open(filename, 'w') as outfile:
                    json.dump(to_dicts(new_data), outfile, indent=2)
            filename = os.path.join(self.base_path, "initial_calibration_M" + master_cam + ".json")
            with open(filename, 'w') as outfile:
                json.dump(to_dicts(new_data), outfile, indent=2)
            pass


    def check_cluster_reprojectionerr(self):
        self.handEye_optimization()
        pass

    def handEye_optimization(self):
        '''
        For only one-to-one group
        '''
        def decode_intrinsic(intrinsic_param):
            dist = intrinsic_param[5:10]
            fx, fy, x0, y0, s = intrinsic_param[0], intrinsic_param[1], intrinsic_param[2], intrinsic_param[3], intrinsic_param[4]
            intrinsic = [
                [fx, s, x0],
                [0, fy, y0],
                [0, 0, 1],
            ]
            return np.array(intrinsic), np.array(dist)
        def find_projection(cam_idx, board_idx, image_list, cam_matrix, cam_dist):
            board = self.workspace.boards[board_idx]
            camera = self.workspace.cameras[cam_idx]
            transformation_list = []
            for img in image_list:
                image_idx = self.workspace.names.image.index(img)

                point_ids = np.flatnonzero(self.workspace.point_table.valid[cam_idx][image_idx][board_idx])
                imagePoints = self.workspace.point_table.points[cam_idx][image_idx][board_idx][point_ids]
                undistorted = camera.undistort_points(imagePoints).astype('float32')
                objPoints = np.array([board.adjusted_points[i] for i in point_ids])

                # detections = self.workspace.detected_points[cam_idx][image_idx][board_idx]
                # undistorted = camera.undistort_points(detections.corners).astype('float32')
                # objPoints = board.points[detections.ids].astype('float32')
                valid, rvec, tvec, error = cv2.solvePnPGeneric(objPoints, undistorted, cam_matrix,
                                                                   cam_dist)
                rt_matrix = np.linalg.inv(rtvec.to_matrix(rtvec.join(rvec[0].flatten(), tvec[0].flatten())))
                transformation_list.append(rt_matrix)
            return np.array(transformation_list)

        def reprojection_error(param_vec, masterCam, groups):
            error = []
            min_error = 100000000
            min_group = None
            for idx, g in enumerate(groups):
                num_camera = len(self.workspace.names.camera)
                slaveCam = self.all_handEye[masterCam][g]['slave_cam']
                slaveBoard = self.all_handEye[masterCam][g]['slave_board']
                masterBoard = self.all_handEye[masterCam][g]['master_board']
                image_list = self.all_handEye[masterCam][g]['image_list']
                slaveCam_idx = self.workspace.names.camera.index(slaveCam)
                slaveBoard_idx = self.workspace.names.board.index(slaveBoard)
                masterCam_idx = self.workspace.names.camera.index(masterCam)
                masterBoard_idx = self.workspace.names.board.index(masterBoard)
                # masterCam_matrix, masterCam_dist = decode_intrinsic(param_vec[masterCam_idx * 10: masterCam_idx * 10 + 10])
                masterCam_wrt_masterB = np.array(self.all_handEye[masterCam][g]['masterCam_wrt_masterB'])
                # masterCam_wrt_masterB1 = find_projection(masterCam_idx, masterBoard_idx, image_list, masterCam_matrix, masterCam_dist)
                # slaveCam_matrix, slaveCam_dist = decode_intrinsic(param_vec[slaveCam_idx*10 : slaveCam_idx*10+10])
                slaveCam_matrix, slaveCam_dist = self.workspace.cameras[slaveCam_idx].intrinsic, self.workspace.cameras[slaveCam_idx].dist

                slaveCam_wrt_masterCam = rtvec.to_matrix(param_vec[0:6])

                masterBoard_t = rtvec.to_matrix(
                    param_vec[(masterBoard_idx * 6) + 6: (masterBoard_idx * 6 + 6) + 6])
                slaveBoard_t = rtvec.to_matrix(
                    param_vec[(slaveBoard_idx * 6) + 6: (slaveBoard_idx * 6 + 6) + 6])

                # slaveB_wrt_masterB = rtvec.to_matrix(param_vec[(idx + 1) * 6:(idx + 1)*6+6])
                slaveB_wrt_masterB = matrix.relative_to(slaveBoard_t, masterBoard_t)
                # slaveCam_wrt_slaveB = find_projection(slaveCam_idx, slaveBoard_idx, image_list, slaveCam_matrix, slaveCam_dist)
                # slaveB_wrt_slaveCam1 = np.linalg.inv(slaveCam_wrt_slaveB)
                slaveB_wrt_slaveCam = np.linalg.inv(np.array(self.all_handEye[masterCam][g]['slaveCam_wrto_slaveB']))
                ZB = matrix.transform(slaveCam_wrt_masterCam, masterCam_wrt_masterB)
                estimated_slaveB_slaveCam = np.linalg.inv(matrix.transform(ZB, np.linalg.inv(slaveB_wrt_masterB)))
                images = self.all_handEye[master_cam][g]['image_list']

                handeye_err = rtvec.from_matrix(estimated_slaveB_slaveCam) - rtvec.from_matrix(slaveB_wrt_slaveCam)
                e = handeye_err.reshape((1,-1)).tolist()[0]

                point_errors, reprojection_error = self.reprojectionError_calculation(slaveCam_idx, slaveBoard_idx,
                                                                                      estimated_slaveB_slaveCam,
                                                                                      images, camMatrix=slaveCam_matrix, camDist=slaveCam_dist)
                if reprojection_error < min_error:
                    min_error = reprojection_error
                    min_group = g
                    error = []
                    error.extend(list(point_errors))
            mse = np.square(np.array(error).ravel()).mean()
            rms = np.sqrt(mse)
            print('min_group: ', min_group)
            print('min_error: ', min_error)
            return error, rms, min_group

        def reprojection_error_eval(param_vec, master_cam, groups, inliers):
            point_errors, _, _ = reprojection_error(param_vec, master_cam, groups)
            errors = np.array([point_errors[i] for i, v in enumerate(inliers) if v == True])
            return errors

        def evaluation(param_vec, masterCam, groups):
            err = []
            for idx,g in enumerate(groups):

                slaveBoard = self.all_handEye[masterCam][g]['slave_board']
                masterBoard = self.all_handEye[masterCam][g]['master_board']

                slaveBoard_idx = self.workspace.names.board.index(slaveBoard)

                masterBoard_idx = self.workspace.names.board.index(masterBoard)

                slaveCam_wrt_masterCam = rtvec.to_matrix(param_vec[0:6])
                masterCam_wrt_masterB = np.array(self.all_handEye[masterCam][g]['masterCam_wrt_masterB'])

                masterBoard_t = rtvec.to_matrix(
                    param_vec[(masterBoard_idx * 6) + 6: (masterBoard_idx * 6 + 6) + 6])
                slaveBoard_t = rtvec.to_matrix(
                    param_vec[(slaveBoard_idx * 6) + 6: (slaveBoard_idx * 6 + 6) + 6])

                slaveB_wrt_masterB = matrix.relative_to(slaveBoard_t, masterBoard_t)
                slaveB_wrt_slaveCam = np.linalg.inv(np.array(self.all_handEye[masterCam][g]['slaveCam_wrto_slaveB']))

                ZB = matrix.transform(slaveCam_wrt_masterCam, masterCam_wrt_masterB)
                estimated_slaveB_slaveCam = np.linalg.inv(matrix.transform(ZB, np.linalg.inv(slaveB_wrt_masterB)))
                error = rtvec.from_matrix(estimated_slaveB_slaveCam) - rtvec.from_matrix(slaveB_wrt_slaveCam)
                err.extend(error.reshape((1,-1)).tolist()[0])
            return err
        pass

        def decode_paramVec(param_vec, master_cam, slave_cam):
            ############ param_vec = self.campose_param[master_cam][idxS*6 : idxS*6+6] + self.board_param  #########
            idxS = self.workspace.names.camera.index(slave_cam)
            cam_pose = rtvec.to_matrix(np.array(param_vec[0:6])).tolist()
            self.campose_param[master_cam][idxS*6 : idxS*6+6] = param_vec[0:6]
            self.cam_pose[master_cam][slave_cam]['extrinsic'] = cam_pose
            board_pose = {}
            for idxb in range(self.workspace.sizes.board):
                pose = np.array(param_vec[idxb*6+6 : idxb*6+12])
                if idxb != 0:
                    board_pose[idxb] = rtvec.to_matrix(rtvec.relative_to(pose, rtvec.from_matrix(board_pose[0])))
                else:
                    board_pose[idxb] = rtvec.to_matrix(rtvec.relative_to(pose, pose))
            self.board_pose_temp.append(board_pose)
            return cam_pose



        cluster_error = {}
        # self.cam_pose2['camera_pose'] = {}
        camera = [self.workspace.names.camera[self.master_cam]]
        for idxM, master_cam in enumerate(self.workspace.names.camera):
            self.final_campose[master_cam] = {}
            # self.cam_pose2['camera_pose'][master_cam] = {}
            # self.cam_pose2['camera_pose'][master_cam][master_cam] = np.eye(4).tolist()
            cluster_error[master_cam] = {}
            # cameraIntrinsic + camera_poses + boards
            # param_vec = self.campose_param[master_cam] + self.board_param
            for idxS, slave_cam in enumerate(self.workspace.names.camera):
                self.final_campose[master_cam][slave_cam]={}
                error = []
                # param_vec = []
                param_vec = self.campose_param[master_cam][idxS*6 : idxS*6+6] + self.board_param
                # board_param = list(np.zeros(len(self.board_param)))
                if master_cam != slave_cam:

                    groups = self.cam_pose[master_cam][slave_cam]['group']
                    '''
                    for boards in self.cam_pose[master_cam][slave_cam]['boards']:
                        boardM = self.workspace.names.board.index(boards[0])
                        boardS = self.workspace.names.board.index(boards[1])
                        board_param[boardM*6 : boardM*6+6] = rtvec.from_matrix(self.board_pose.poses[boardM])
                        board_param[boardS * 6: boardS * 6 + 6] = rtvec.from_matrix(self.board_pose.poses[boardS])
                    param_vec += board_param
                    '''
                    err1 = evaluation(np.array(param_vec), master_cam, groups)
                    point_errors1, rms1, min_group1 = reprojection_error(np.array(param_vec), master_cam, groups)
                    self.final_campose[master_cam][slave_cam]['initial_reprojectionError'] = rms1
                    groups = [min_group1]
                    #### optimization using handEye equation
                    res = least_squares(evaluation, param_vec,
                                        verbose=2, x_scale='jac', f_scale=1.0, ftol=1e-4, max_nfev=100,
                                        method='trf', loss='linear', args=(master_cam, groups))
                    err2 = evaluation(res.x, master_cam, groups)
                    # decode_paramVec(list(res.x), master_cam, slave_cam)

                    point_errors2, rms2, min_group2 = reprojection_error(res.x, master_cam, groups)
                    groups = [min_group2]
                    inliers = np.ones(len(point_errors2))
                    for i in range(1):
                        info(f"Adjust_outliers {i}:")
                        res = least_squares(reprojection_error_eval, res.x,
                                            verbose=2, x_scale='jac', f_scale=1.0, ftol=1e-4, max_nfev=100,
                                            method='trf', loss='linear', args=(master_cam, groups, inliers))
                        point_errors0 = np.array(reprojection_error_eval(res.x, master_cam, groups, inliers), dtype=object)
                        mse = np.square(np.array(point_errors0).ravel()).mean()
                        rms = np.sqrt(mse)

                        point_errors, _, _ = np.array(reprojection_error(res.x, master_cam, groups), dtype=object)
                        threshold = np.quantile(point_errors0, 0.70)
                        inliers = np.array(point_errors < threshold).flatten()
                    point_errors, rms, min_group = reprojection_error(res.x, master_cam, groups)
                    self.final_campose[master_cam][slave_cam]['final_reprojectionError'] = rms
                    self.final_campose[master_cam][slave_cam]['group'] = min_group
                    self.final_campose[master_cam][slave_cam]['cam_pose'] = rtvec.to_matrix(np.array(res.x[0:6])).tolist()
                    # self.draw_viz(master_cam, min_group, rms)
                    # self.cam_pose2['camera_pose'][master_cam][slave_cam] = rtvec.to_matrix(res.x[0:6]).tolist()

        # self.calculate_board_mean()
        # self.final_reprojection_error()
        pass

    def draw_viz(self, master_cam, group, final_error):
        masterBoard_angles = self.all_handEye[master_cam][group]['masterBoard_angle']
        slaveBoard_angles = self.all_handEye[master_cam][group]['slaveBoard_angle']
        masterBoard_error = self.all_handEye[master_cam][group]['masterBoard_error']
        slaveBoard_error = self.all_handEye[master_cam][group]['slaveBoard_error']
        master_x = []
        master_y = []
        master_z = []
        master_name = []
        slave_x = []
        slave_y = []
        slave_z = []
        slave_name = []
        for k in masterBoard_angles.keys():
            master_x.append(masterBoard_angles[k][0])
            master_y.append(masterBoard_angles[k][1])
            master_z.append(masterBoard_error[k])
            master_name.append(k)
            slave_x.append(slaveBoard_angles[k][0])
            slave_y.append(slaveBoard_angles[k][1])
            slave_z.append(slaveBoard_error[k])
            slave_name.append(k)

        dataM = {'view_angle X': master_x, 'view_angle Y': master_y, 'reprojection error': master_z, 'name': master_name}
        dataS = {'view_angle X': slave_x, 'view_angle Y': slave_y, 'reprojection error': slave_z,
                 'name': slave_name}

        nameM = master_cam +' group-'+ str(group)
        final_layout = go.Figure()
        final_layout.add_trace(
            go.Scatter3d(
                x=master_x,
                y=master_y,
                z=master_z,
                name=nameM
            )
        )
        nameS = self.all_handEye[master_cam][group]['slave_cam']
        final_layout.add_trace(
            go.Scatter3d(
                x=slave_x,
                y=slave_y,
                z=slave_z,
                name=nameS
            )
        )

        final_layout.show()
        pass

    def final_reprojection_error(self):
        def fun(param_vec):
            error_list = []
            for img in range(0, 10):
                for slaveCam in range(self.workspace.sizes.camera):
                    cameraMatrix, distortion = self.workspace.cameras[slaveCam].intrinsic, self.workspace.cameras[
                        slaveCam].dist
                    masterCam_wrto_slaveCam = np.linalg.inv(rtvec.to_matrix(np.array(param_vec[slaveCam*6 : slaveCam*6+6])))
                    if slaveCam == self.master_cam:
                        master_boards = np.flatnonzero(self.workspace.pose_table.valid[slaveCam][img])
                    for boardS in range(self.workspace.sizes.board):
                        boardS_pose = np.array(param_vec[self.workspace.sizes.camera*6+boardS*6 : self.workspace.sizes.camera*6+boardS*6+6])
                        for boardM in master_boards:
                            boardM_pose = np.array(param_vec[self.workspace.sizes.camera*6+boardM*6 : self.workspace.sizes.camera*6+boardM*6+6])
                            slaveboard_wrto_boardM = rtvec.to_matrix(rtvec.relative_to(boardS_pose, boardM_pose))
                            boardM_wrto_masterCam = self.workspace.pose_table.poses[self.master_cam][img][boardM]
                            ZB = matrix.transform(boardM_wrto_masterCam, masterCam_wrto_slaveCam)
                            estimated_slaveboard_wrto_slaveCam = matrix.transform(slaveboard_wrto_boardM, ZB)
                            slaveboard_wrto_slaveCam = self.workspace.pose_table.poses[slaveCam][img][boardS]

                            point_ids = np.flatnonzero(self.workspace.point_table.valid[slaveCam][img][boardS])
                            if point_ids.size != 0:
                                imagePoints = self.workspace.point_table.points[slaveCam][img][boardS][point_ids]
                                objectPoints = np.array(
                                    [self.workspace.boards[boardS].adjusted_points[i] for i in point_ids])

                                rvec, tvec = rtvec.split(rtvec.from_matrix(estimated_slaveboard_wrto_slaveCam))
                                imP, _ = cv2.projectPoints(np.copy(objectPoints), rvec, tvec, cameraMatrix, distortion)
                                error = np.linalg.norm(imagePoints - imP.reshape([-1, 2]), axis=-1)
                                # point_errors.extend(imagePoints - imP.reshape([-1, 2]))
                                error_list.extend(list(error))
            return error_list

        def evaluation(param_vec, inliers, x):
            point_errors = fun(param_vec)
            errors = np.array([point_errors[i] for i, v in enumerate(inliers) if v == True])
            return errors

        master_cam = self.workspace.names.camera[self.master_cam]
        param_vec = self.campose_param[master_cam] + self.board_param
        point_errors = fun(param_vec)
        inliers = np.ones(len(point_errors))
        for i in range(1):
            info(f"Adjust_outliers {i}:")
            res = least_squares(evaluation, param_vec,
                                verbose=2, x_scale='jac', f_scale=1.0, ftol=1e-4, max_nfev=100,
                                method='trf', loss='linear', args=(inliers, 0))
            point_errors = fun(res.x)
            threshold = np.quantile(point_errors, 0.70)
            inliers = np.array(point_errors < threshold).flatten()
        pass

    def calculate_board_mean(self):
        all_pose = {}
        for idxg, group in enumerate(self.board_pose_temp):
            for idxb in range(self.workspace.sizes.board):
                if idxb not in all_pose:
                    all_pose[idxb] = rtvec.from_matrix(self.board_pose_temp[idxg][idxb]).reshape((1,-1))
                else:
                    pose = rtvec.from_matrix(self.board_pose_temp[idxg][idxb]).reshape((1,-1))
                    all_pose[idxb] = np.concatenate((all_pose[idxb], pose), axis=0)

        for idxb in range(self.workspace.sizes.board):
            x = rtvec.to_matrix(common.mean_robust(all_pose[idxb]))
            self.board_pose.poses[idxb] = x.tolist()

        pass

    def optimization(self, slave_cam, boardS, image_list,
                     init_slaveCam_wrt_masterCam, init_slaveB_wrt_masterB, masterCam_wrt_masterB, slaveCam_wrt_slaveB, num_adjustments):
        def decode_intrinsic(intrinsic_param):
            dist = intrinsic_param[5:10]
            fx, fy, x0, y0, s = intrinsic_param[0], intrinsic_param[1], intrinsic_param[2], intrinsic_param[3], intrinsic_param[4]
            intrinsic = [
                [fx, s, x0],
                [0, fy, y0],
                [0, 0, 1],
            ]
            return np.array(intrinsic), np.array(dist)

        def fun(param_vec, camMatrix, camDist):
            slaveCam_wrt_masterCam = rtvec.to_matrix(param_vec[0:6])
            slaveB_wrt_masterB = rtvec.to_matrix(param_vec[6:12])
            ZB = matrix.transform(slaveCam_wrt_masterCam, masterCam_wrt_masterB)
            estimated_slaveB_slaveCam = np.linalg.inv(matrix.transform(ZB, np.linalg.inv(slaveB_wrt_masterB)))
            point_errors, reprojection_error = self.reprojectionError_calculation(slave_cam, boardS,
                                                                                  estimated_slaveB_slaveCam, image_list,
                                                                                  camMatrix=camMatrix, camDist=camDist)
            return point_errors, reprojection_error

        def evaluation(param_vec, camMatrix, camDist, inliers):
            point_errors, _ = fun(param_vec, camMatrix, camDist)
            errors = np.array([point_errors[i] for i, v in enumerate(inliers) if v == True ])
            return errors

        ### new
        slaveCam_vec = rtvec.from_matrix(init_slaveCam_wrt_masterCam)
        slaveB_vec = rtvec.from_matrix(init_slaveB_wrt_masterB)
        intrinsic = np.array(self.camintrinsic_param[slave_cam*10: slave_cam*10+10])
        K, dist = decode_intrinsic(intrinsic)
        param_vec = np.concatenate((slaveCam_vec, slaveB_vec))
        point_errors, _ = fun(param_vec, K, dist)
        inliers = np.ones(len(point_errors))

        if num_adjustments ==0:
            return 0
        for i in range(num_adjustments):
            info(f"Adjust_outliers {i}:")
            res = least_squares(evaluation, param_vec,
                                verbose=2, x_scale='jac', f_scale=1.0, ftol=1e-4, max_nfev=100,
                                method='trf', loss='linear', args=(K, dist, inliers))
            point_errors, _ = fun(param_vec, K, dist)
            threshold = np.quantile(point_errors, 0.75)
            inliers = np.array(point_errors < threshold).flatten()

        ### new
        _, mean_error = fun(res.x, K, dist)
        return mean_error

    def reprojectionError_calculation(self, slaveCam, slaveBoard, estimated_slaveBoard_slaveCam, image_list, camMatrix=np.zeros((3,3)), camDist=np.zeros((1,5))):
        '''
        for Master_cam to Slave_cam
        '''
        error_list = []
        # T1 = matrix.transform(base_wrt_gripper, board_wrt_boardM)
        # T2 = matrix.transform(np.linalg.inv(slave_wrt_master), T1)
        if camMatrix.all() == 0 and camDist.all() == 0:
            cameraMatrix = self.workspace.cameras[slaveCam].intrinsic
            distortion = self.workspace.cameras[slaveCam].dist
        else:
            cameraMatrix = camMatrix
            distortion = camDist
        images = self.workspace.names.image
        point_errors = []
        for i in range(len(image_list)):
            point_ids = np.flatnonzero(self.workspace.point_table.valid[slaveCam][images.index(image_list[i])][slaveBoard])
            if point_ids.size != 0:
                imagePoints = self.workspace.point_table.points[slaveCam][images.index(image_list[i])][slaveBoard][point_ids]
                # undistorted = self.workspace.cameras[slaveCam].undistort_points(imagePoints).astype('float32')
                objectPoints = np.array([self.workspace.boards[slaveBoard].adjusted_points[i] for i in point_ids])
                rmatrix = estimated_slaveBoard_slaveCam[i][0:3, 0:3]
                tmatrix = estimated_slaveBoard_slaveCam[i][0:3, 3]
                rvec = (R.from_matrix([rmatrix]).as_rotvec())
                tvec = (tmatrix.T)
                imP, _ = cv2.projectPoints(np.copy(objectPoints), rvec, tvec, cameraMatrix, distortion)
                error = np.linalg.norm(imagePoints - imP.reshape([-1, 2]), axis=-1)
                point_errors.extend(imagePoints - imP.reshape([-1, 2]))
                # mse = np.square(error).mean()
                # rms = np.sqrt(mse)
                error_list.extend(list(error))
        # new
        mse = np.square(np.array(point_errors).ravel()).mean()
        rms = np.sqrt(mse)
        # new
        return error_list, rms

    def check_Rotation_Orthogonality(self, rotation_matrix, img):
        M = rotation_matrix @ rotation_matrix.T
        check = False
        x = int(np.linalg.det(M))
        if x == 1:
            check = True
        else:
            pass
            # print('non orthogonal image: ', img)
        return check

    def check_handEye_Cluster(self, master_rvec, slave_rvec, image_list, cluster_bandwidth=8, element_limit=2):

        deleted_ImageId = []
        for vec in [master_rvec, slave_rvec]:
            master_mean_shift = MeanShift().fit(np.array(vec))
            meanshift_classification = master_mean_shift.predict(np.array(vec))
            unique_cluster = np.unique(np.array(meanshift_classification))
            meanshift_cluster = unique_cluster.shape[0]
            for c in unique_cluster:
                count = list(meanshift_classification).count(c)
                if count < element_limit:
                    array = np.array(meanshift_classification)
                    index = np.where(array == c)[0]
                    for idx in index:
                        if not deleted_ImageId:
                            deleted_ImageId.append(idx)
                        elif idx not in deleted_ImageId:
                            deleted_ImageId.append(idx)
                    # deleted_ImageId.extend(list(index))

        if deleted_ImageId:
            for i in sorted(deleted_ImageId, reverse=True):
                del image_list[i]

        return image_list

        pass

    def master_slave_pose(self, master_cam, master_board, slave_cam, slave_board, check_cluster = True):
        num_images = len(self.workspace.names.image)
        '''
        For Master_cam to Slave_cam
        :return -> Rt of MasterCam_to_MasterBoard
                   Rt of SlaveCam_to_SlaveBoard
                   image_list
        '''
        masterR_list = []
        masterT_list = []
        slaveR_list = []
        slaveT_list = []
        image_list = []
        master_rvec_list = []
        slave_rvec_list = []
        masterTransformation_dict = {}
        slaveTransformation_dict = {}
        for idx, img in enumerate(self.workspace.names.image):
            master_valid = self.workspace.pose_table.valid[master_cam][idx][master_board]
            slave_valid = self.workspace.pose_table.valid[slave_cam][idx][slave_board]
            if master_valid and slave_valid:
                master_pose = np.linalg.inv(self.workspace.pose_table.poses[master_cam][idx][master_board])
                master_R, master_t = matrix.split(master_pose)
                master_check = self.check_Rotation_Orthogonality(master_R, img)
                slave_pose = np.linalg.inv(self.workspace.pose_table.poses[slave_cam][idx][slave_board])
                slave_R, slave_t = matrix.split(slave_pose)
                slave_check = self.check_Rotation_Orthogonality(slave_R, img)
                master_rvec, master_tvec = rtvec.split(rtvec.from_matrix(master_pose))
                slave_rvec, slave_tvec = rtvec.split(rtvec.from_matrix(slave_pose))
                master_rvec_list.append(master_rvec)
                slave_rvec_list.append(slave_rvec)
                masterR_list.append(master_R)
                masterT_list.append(master_t)
                slaveR_list.append(slave_R)
                slaveT_list.append(slave_t)
                image_list.append(img)
                masterTransformation_dict[img] = master_pose
                slaveTransformation_dict[img] = slave_pose

        if check_cluster:
            if len(image_list)>6:
                final_image_list = self.check_handEye_Cluster(master_rvec_list, slave_rvec_list, image_list)
                for img in final_image_list:
                    mR, mT = matrix.split(masterTransformation_dict[img])
                    sR, sT = matrix.split(slaveTransformation_dict[img])
                    masterR_list.append(mR)
                    masterT_list.append(mT)
                    slaveR_list.append(sR)
                    slaveT_list.append(sT)

        return np.array(masterR_list), np.array(masterT_list), np.array(slaveR_list), np.array(slaveT_list), image_list

    def export_handEye_Camera(self):
        filename = os.path.join(self.base_path, "handEyeCamera.json")
        json_object = json.dumps((self.all_handEye), indent=4)
        # Writing to sample.json
        with open(filename, "w") as outfile:
            outfile.write(json_object)
        pass
        v = Interactive_Extrinsic(base_path)


    def export_camera_board_param(self):
        param = {}
        param['camera_pose'] = self.cam_pose
        param['board_pose'] = self.board_pose
        filename = os.path.join(self.base_path, "CameraBoard_param.json")
        json_object = json.dumps((param), indent=4)
        # Writing to sample.json
        with open(filename, "w") as outfile:
            outfile.write(json_object)
        pass

def main1(base_path, limit_image=10):
    '''
    Input->  base_path : It takes folder containing only 1 camera, board.yaml, calibration.json, gripper_pose.json
             cam : int (choice 0)
             board : int (choice 0,1,2,3,4)
    Output->  Create handEyeGripper.json file containing relation between that camera and one board
    '''
    h = handEye(base_path)
    h.initiate_workspace()
    h.set_gripper_pose()
    # h.viewPlanRobust()
    h.viewPlanSimple()

    # h.export_handEyeGripper(handEye_struct)
    # h.estimate_camera_board_poses()

def main2(base_path, master="cam218"):
    """
    Input-> base_path: folder with 2 Cameras
    :return:
    """
    handEye_dict = {}
    for path, subdirs, files in os.walk(base_path):
        if path == base_path:
            for dir in subdirs:
                cam_path = os.path.join(base_path, dir)
                h = handEye(cam_path)
                h.initiate_workspace()
                h.set_gripper_pose()
                handEye_dict[dir] = h

    for key in handEye_dict:
        if key == master:
            handEyeGripper = json.load(open(handEye_dict[key].handEyeGripper))
            masterCam = handEyeGripper['camera']
            masterBoard = int(handEyeGripper['board'][-1])
            base_wrt_masterCam = handEyeGripper['base_wrt_cam']
        else:
            handEyeGripper = json.load(open(handEye_dict[key].handEyeGripper))
            slaveCam = handEyeGripper['camera']
            slaveBoard = int(handEyeGripper['board'][-1])
            gripper_wrt_slaveBoard = handEyeGripper['gripper_wrt_world']

    slaveBoard_masterCam_pose = handEye_dict[master].workspace.pose_table._index_select(0, axis=0)._index_select(slaveBoard, axis=1)
    images = handEye_dict[master].workspace.names.image
    gripper_pose = handEye_dict[master].gripper_pose

    valid_poses = [idx for idx, valid in enumerate(slaveBoard_masterCam_pose.valid) if valid == True]

    error_dict = {}
    for p in valid_poses:
        img_name = images[p]
        grip_pose = img_name[:-4][1:]
        gripper_wrt_base = gripper_pose[grip_pose]
        board_wrt_cam = slaveBoard_masterCam_pose.poses[p]
        masterCam_wrt_slaveBoard = np.linalg.inv(board_wrt_cam)
        ZB = matrix.transform(base_wrt_masterCam, masterCam_wrt_slaveBoard)
        error_dict[p] = np.linalg.inv(gripper_wrt_base) - matrix.transform(ZB, np.linalg.inv(gripper_wrt_slaveBoard))
    pass

def main3(base_path, limit_images, num_adjustments):
    """
    For Camera to Camera handEye
    :return:
    """
    h = handEye(base_path)
    h.initiate_workspace()
    # all_handEye = {}
    for idx, master_cam in enumerate(h.workspace.names.camera):
        handEye_dict = h.handEye_table(master_cam=idx, limit_image=limit_images, num_adjustments=num_adjustments)
        h.all_handEye[master_cam] = handEye_dict
    # handEye_dict = h.handEye_table(master_cam=master_cam)
    h.export_handEye_Camera()
    pass

def main4(base_path, limit_images, num_adjustments, limit_board_image, calculate_handeye, check_cluster):

    h = handEye(base_path)
    h.initiate_workspace()
    h.calc_camPose_param(limit_images, num_adjustments, limit_board_image, calculate_handeye, check_cluster)
    h.export_handEye_Camera()
    # h.check_cluster_reprojectionerr()

    pass

def main5(base_path, limit_images, num_adjustments):

    h = handEye(base_path)
    h.initiate_workspace()
    handEye_dict = h.handEye_table(master_cam=1, limit_image=limit_images, num_adjustments=num_adjustments)
    pass

def main6(base_path, limit_images, num_adjustments):

    h = handEye(base_path)
    h.initiate_workspace(show_all_poses=True)
    pass

if __name__ == '__main__':
    # main1(base_path, limit_image=10)
    # main3(base_path, limit_images=10, num_adjustments=2)
    main4(base_path, limit_images=6, num_adjustments=0, limit_board_image=6, calculate_handeye=True, check_cluster=False)
    # main5(base_path, limit_images=10, num_adjustments=1)
    # main6(base_path, limit_images=10, num_adjustments=1)
    pass