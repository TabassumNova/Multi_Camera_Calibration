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


class handEye():
    def __init__(self, base_path, master_cam=0, master_board=0):
        self.base_path = base_path
        self.master_cam = master_cam
        self.master_board = master_board
        self.datasetPath = base_path
        self.boardPath = None
        self.poseJsonPath = None
        self.intrinsicPath = None
        self.workspace = None
        self.gripper_pose = {}
        self.collect_files()
        self.board_pose = {}
        self.board_pose_temp = []
        self.cam_pose = None
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
        It takes files:
        boards.yaml -> Board config
        calibration.json -> Camera Intrinsic,
        gripper_pose.json -> Gripper pose
        """
        for path, subdirs, files in os.walk(self.base_path):
            for name in files:
                if name == 'boards.yaml':
                    self.boardPath = os.path.join(self.base_path, name)
                elif name == 'calibration.json':
                    self.intrinsicPath = os.path.join(self.base_path, name)
                elif name == 'gripper_pose.json':
                    self.poseJsonPath = os.path.join(self.base_path, name)


    def export_workspace(self):
        workspace_pickle = os.path.join(self.base_path, "workspace.pkl")
        with open(workspace_pickle, "wb") as file:
            pickle.dump(self.workspace, file)

    def camera_intrinsic_dataset(self):
        '''
        This function exports "intrinsic_dataset.json" which contains the image lists that
        were used during Intrinsic Calculation
        - this file is mainly needed for GUI camera window
        '''
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

    def initiate_workspace(self, show_all_poses=True):
        '''
        This funtion is a gateway to Multical
        It receives detections, Intrinsic calculation and pose estimation from
        Multical
        '''
        pathO = args.PathOpts(image_path=self.datasetPath)
        cam = args.CameraOpts(calibration=self.intrinsicPath, intrinsic_error_limit=0.5)
        pose_estimation_method = "solvePnPGeneric"
        runt = args.RuntimeOpts(pose_estimation=pose_estimation_method, show_all_poses=show_all_poses)
        opt = args.OptimizerOpts(outlier_threshold=1.2, fix_intrinsic=True, adjust_outliers=False)
        c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
        self.workspace = c.execute_board()
        self.export_workspace()
        if not self.intrinsicPath:
            self.camera_intrinsic_dataset()
        self.set_Cam_color()

    def handEye_gripper(self, camera, board):
        '''
        This function is for moving one board with respect to one camera
        It generates transformation for Camera_to_Board and Gripper_to_Board
        '''
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


    def set_gripper_pose(self):
        '''
        It takes gripper_pose.json and generates a dict from it
        '''
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
        '''
        handEyeGripper.json contains info for moving one board wrto one camera
        '''
        json_dict = {}
        json_dict['camera'] = handEye_struct.camera
        json_dict['board'] = handEye_struct.board
        json_dict["base_wrt_cam"] = handEye_struct.base_wrt_cam.tolist()
        json_dict["gripper_wrt_world"] = handEye_struct.gripper_wrt_world.tolist()
        json_object = json.dumps(json_dict, indent=4)
        # Writing to sample.json
        handEyeGripper_path = os.path.join(self.base_path, "handEyeGripper.json")
        with open(handEyeGripper_path, "w") as outfile:
            outfile.write(json_object)
        pass

    # @jit(target_backend='cuda')
    def handEye_table(self, master_cam=0, limit_image=6, limit_board_image=6, calculate_handeye=True, check_cluster=False):
        '''
        handEye for Master_cam to Slave_cam
        :param limit_image: number of image allowed for hand eye pair
               limit_board_image : number of image allowed to choose as major board seen by cameras
        :return: It generates all possible groups for master_cam and slave_cam pair
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
                    for boardS in slave_boards:
                        masterR, masterT, slaveR, slaveT, image_list = self.master_slave_pose(master_cam, boardM,
                                                                                         slave_cam, boardS, check_cluster)

                        if len(image_list)>limit_image:
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

                            masterBoard_pose, slaveBoard_pose, masterBoard_angle, slaveBoard_angle, masterBoard_error, slaveBoard_error = self.collect_pose_angles(master_cam, boardM, slave_cam, boardS, image_list)

                            handEye_dict[serial] = to_dicts(struct(master_cam=self.workspace.names.camera[master_cam],
                                                        master_board=self.workspace.names.board[boardM],
                                                        slave_cam = self.workspace.names.camera[slave_cam],
                                                        slave_board=self.workspace.names.board[boardS],
                                                        masterCam_wrt_masterB=masterCam_wrt_masterB.tolist(),
                                                        boardS_wrto_boardM=slaveB_wrt_masterB.tolist(),
                                                        slaveCam_wrto_masterCam=slaveCam_wrt_masterCam.tolist(),
                                                        slaveCam_wrto_slaveB = slaveCam_wrt_slaveB.tolist(),
                                                        estimated_slaveB_slaveCam = estimated_slaveB_slaveCam.tolist(),
                                                        # initial_reprojection_error=str(initial_error), final_reprojection_error=str(final_error),
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
        '''
        Exports "meanCameras.json" which contains initial transformations of the cameras
        '''
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
            self.board_pose = self.workspace.board_poses
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
        # b = Interactive_Extrinsic_Board(self.calibObj_mean, self.workspace.names.board)
        pass

    def calc_camPose_param(self, limit_images=6, limit_board_image=6, calculate_handeye=True, check_cluster=False):
        cameras = self.workspace.names.camera
        # cameras = ['08320217']
        for idx, master_cam in enumerate(cameras):
            handEye_dict = self.handEye_table(master_cam=idx, limit_image=limit_images,
                                              limit_board_image=limit_board_image,
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
        # v = Interactive_Extrinsic(self.base_path)


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
    h.viewPlanSimple()


def main4(base_path, limit_images=6, limit_board_image=6, calculate_handeye=True, check_cluster=False):

    h = handEye(base_path)
    h.initiate_workspace()
    h.calc_camPose_param(limit_images, limit_board_image, calculate_handeye, check_cluster)
    h.export_handEye_Camera()

# if __name__ == '__main__':
#     base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V35"
#     # main1(base_path, limit_image=10)
#     main4(base_path, limit_images=6, limit_board_image=6, calculate_handeye=True, check_cluster=False)
#     pass