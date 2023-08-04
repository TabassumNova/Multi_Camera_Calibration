import math

import numpy as np

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

base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\isohedron\V31"

class handEye():
    def __init__(self, base_path):
        self.base_path = base_path
        self.datasetPath = base_path
        self.boardPath = None
        self.poseJsonPath = None
        self.intrinsicPath = None
        self.workspace = None
        self.handEyeGripper = None
        self.gripper_pose = {}
        self.collect_files()
        self.board_pose = {}
        self.cam_pose = None
        self.cam_pose2 = {}
        self.board_pose2 = {}
        self.all_handEye = {}
        self.board_param = []
        self.campose_param = {}
        self.camintrinsic_param = []
        self.inliers = None

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


    def initiate_workspace(self, show_all_poses=False):
        pathO = args.PathOpts(image_path=self.datasetPath)
        cam = args.CameraOpts(motion_model="calibrate_board", intrinsic_error_limit=0.5)
        # pose_estimation_method = "solvePnPRansac"
        pose_estimation_method = "solvePnPGeneric"
        runt = args.RuntimeOpts(pose_estimation=pose_estimation_method, show_all_poses=show_all_poses)
        opt = args.OptimizerOpts(outlier_threshold=1.2, fix_intrinsic=True)
        c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
        self.workspace = c.execute_board()
        # self.workspace.pose_table = make_pose_table(self.workspace.point_table, self.workspace.boards,
        #                                             self.workspace.cameras, method="solvePnPRansac")

        self.workspace.point_table.valid = self.workspace.pose_table.inliers

        for c in range(len(self.workspace.names.camera)):
            self.camintrinsic_param.extend(self.workspace.cameras[c].param_vec.tolist())

        # self.refine_intrinsic()

        for b in range(len(self.workspace.names.board)):
            t = rtvec.from_matrix(self.workspace.board_poses.poses[b])
            self.board_param.extend(rtvec.from_matrix(self.workspace.board_poses.poses[b]).tolist())





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
        base_wrt_cam, gripper_wrt_world, camera_wrt_world, base_wrt_gripper, estimated_gripper_base, _, _ = self.hand_eye_robot_world(np.array(R_cam2world_list),
                                            np.array(t_cam2world_list), np.array(R_base2gripper_list), np.array(t_base2gripper_list))

        handEye_struct = struct(camera=self.workspace.names.camera[camera], board=self.workspace.names.board[board], base_wrt_cam=base_wrt_cam, gripper_wrt_world=gripper_wrt_world,
                                camera_wrt_world=camera_wrt_world, base_wrt_gripper=base_wrt_gripper, estimated_gripper_base=estimated_gripper_base, images=image_list,
                                corners=corner_list, objPoints=point3D_list)

        self.reprojectionError_Calculation_new(handEye_struct)
        self.test_robotMove(handEye_struct)

        return handEye_struct

    def hand_eye_robot_world(self, cam_world_R, cam_world_t, base_gripper_R, base_gripper_t):
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

    def handEye_table(self, master_cam=0, limit_image=2, num_adjustments=5):
        '''
        handEye for Master_cam to Slave_cam
        :param master_cam:
        :return:
        '''
        num_cameras = len(self.workspace.names.camera)
        num_boards = len(self.workspace.names.board)

        handEye_dict = {}
        serial = 0
        master_boards = [b for b in range(0, num_boards) if self.workspace.pose_table.valid[master_cam][:, b].sum() > 20]
        for slave_cam in range(0, num_cameras):
            slave_boards = [b for b in range(0, num_boards) if self.workspace.pose_table.valid[slave_cam][:, b].sum() > 20]

            for boardM in master_boards:
                if boardM not in self.board_pose:
                    self.board_pose[boardM] = {}

                for boardS in slave_boards:
                    masterR, masterT, slaveR, slaveT, image_list = self.master_slave_pose(master_cam, boardM,
                                                                                     slave_cam, boardS)

                    if len(image_list)>limit_image:
                        # board_wrt_boardM, slave_wrt_master, world_wrt_camera, \
                        # base_wrt_gripper, err, err2 = hand_eye.hand_eye_robot_world(masterR, masterT, slaveR, slaveT)
                        print("MasterCam: ",master_cam, " MasterB: ", boardM, " SlaveCam: ", slave_cam, " SlaveB: ", boardS, " images: ", image_list)
                        slaveCam_wrt_masterCam, slaveB_wrt_masterB, masterCam_wrt_masterB, slaveCam_wrt_slaveB, \
                        estimated_slaveB_slaveCam, err, err2 = self.hand_eye_robot_world(masterR, masterT, slaveR, slaveT)

                        if boardS not in self.board_pose[boardM]:
                            self.board_pose[boardM][boardS] = slaveB_wrt_masterB.tolist()

                        _, initial_error = self.reprojectionError_calculation(slave_cam, boardS, estimated_slaveB_slaveCam, image_list)
                        # error = sum(reprojection_error)/len(reprojection_error)
                        print("initial reprojection error: ", initial_error)
                        final_error = self.optimization(slave_cam, boardS, image_list,
                                                    slaveCam_wrt_masterCam, slaveB_wrt_masterB,
                                                    masterCam_wrt_masterB, slaveCam_wrt_slaveB, num_adjustments=num_adjustments)
                        handEye_dict[serial] = to_dicts(struct(master_cam=self.workspace.names.camera[master_cam],
                                                    master_board=self.workspace.names.board[boardM],
                                                    slave_cam = self.workspace.names.camera[slave_cam],
                                                    slave_board=self.workspace.names.board[boardS],
                                                    boardS_wrto_boardM=slaveB_wrt_masterB.tolist(),
                                                    slaveCam_wrto_masterCam=slaveCam_wrt_masterCam.tolist(),
                                                    slaveCam_wrto_slaveB = slaveCam_wrt_slaveB.tolist(),
                                                    estimated_slaveB_slaveCam = estimated_slaveB_slaveCam.tolist(),
                                                    initial_reprojection_error=str(initial_error), final_reprojection_error=str(final_error),
                                                               image_list=image_list, masterCam_wrt_masterB=masterCam_wrt_masterB.tolist()))
                        serial += 1

        return handEye_dict

    def show_cluster_mean(self, mean_calculation):
        mean_group_dict = {}
        for cam1, cam_value1 in mean_calculation.items():
            mean_group_dict[cam1] = {}
            mean_group_dict[cam1][cam1] = {}
            mean_group_dict[cam1][cam1]['extrinsic'] = np.eye(4).tolist()
            mean_group_dict[cam1][cam1]['group'] = [0]
            for cam2, cam_value2 in cam_value1.items():
                if cam2 != cam1:
                    g = common.cluster(cam_value2['extrinsic'])
                    group = np.array(cam_value2['group'])[common.cluster(cam_value2['extrinsic'])]
                    x = rtvec.to_matrix(common.mean_robust(cam_value2['extrinsic']))
                    mean_group_dict[cam1][cam2] = {}
                    mean_group_dict[cam1][cam2]['extrinsic'] = x.tolist()
                    mean_group_dict[cam1][cam2]['group'] = group.tolist()
        return mean_group_dict

    def calc_camPose_param(self, limit_images, num_adjustments):
        cameras = self.workspace.names.camera
        # cameras = ['08320217']
        for idx, master_cam in enumerate(cameras):
            handEye_dict = self.handEye_table(master_cam=idx, limit_image=limit_images, num_adjustments=num_adjustments)
            self.all_handEye[master_cam] = handEye_dict

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
        self.cam_pose = self.show_cluster_mean(mean_calculation)
        for master, master_value in self.cam_pose.items():
            self.campose_param[master] = []
            for slave in self.workspace.names.camera:
                self.campose_param[master].extend(rtvec.from_matrix(np.array(master_value[slave]['extrinsic'])).tolist())
        pass

    def check_cluster_reprojectionerr(self):
        self.handEye_optimization()

        pass

    def handEye_optimization(self):
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
                detections = self.workspace.detected_points[cam_idx][image_idx][board_idx]
                undistorted = camera.undistort_points(detections.corners).astype('float32')
                objPoints = board.points[detections.ids].astype('float32')
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
                slaveCam_wrt_masterCam = rtvec.to_matrix(param_vec[(slaveCam_idx*6):(slaveCam_idx*6)+6])

                masterBoard_t = rtvec.to_matrix(param_vec[(masterBoard_idx * 6) + num_camera * 6: (
                                                masterBoard_idx * 6 + 6) + num_camera * 6])
                slaveBoard_t = rtvec.to_matrix(param_vec[(slaveBoard_idx * 6) + num_camera * 6: (
                                                slaveBoard_idx * 6 + 6)  + num_camera * 6])
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
                num_camera = len(self.workspace.names.camera)
                slaveCam = self.all_handEye[masterCam][g]['slave_cam']
                slaveBoard = self.all_handEye[masterCam][g]['slave_board']
                masterBoard = self.all_handEye[masterCam][g]['master_board']
                image_list = self.all_handEye[masterCam][g]['image_list']
                slaveCam_idx = self.workspace.names.camera.index(slaveCam)
                slaveBoard_idx = self.workspace.names.board.index(slaveBoard)
                # slaveCam_matrix, slaveCam_dist = decode_intrinsic(param_vec[slaveCam_idx * 10: slaveCam_idx * 10 + 10])
                slaveCam_matrix, slaveCam_dist = self.workspace.cameras[slaveCam_idx].intrinsic, self.workspace.cameras[
                    slaveCam_idx].dist
                # masterCam_idx = self.workspace.names.camera.index(masterCam)
                masterBoard_idx = self.workspace.names.board.index(masterBoard)
                # masterCam_matrix, masterCam_dist = decode_intrinsic(
                #     param_vec[masterCam_idx * 10: masterCam_idx * 10 + 10])

                slaveCam_wrt_masterCam = rtvec.to_matrix(param_vec[(slaveCam_idx*6):(slaveCam_idx*6)+6])
                masterCam_wrt_masterB = np.array(self.all_handEye[masterCam][g]['masterCam_wrt_masterB'])
                # masterCam_wrt_masterB1 = find_projection(masterCam_idx, masterBoard_idx, image_list, masterCam_matrix,
                #                                          masterCam_dist)
                y = param_vec[(masterBoard_idx*6)+num_camera*6 : (masterBoard_idx*6+6)+num_camera*6]
                masterBoard_t = rtvec.to_matrix(param_vec[(masterBoard_idx*6)+num_camera*6 : (masterBoard_idx*6+6)+num_camera*6])
                slaveBoard_t = rtvec.to_matrix(param_vec[(slaveBoard_idx * 6) + num_camera*6: (slaveBoard_idx * 6 + 6) + num_camera*6])
                # slaveB_wrt_masterB = rtvec.to_matrix(param_vec[(idx + 1) * 6:(idx + 1)*6+6])
                slaveB_wrt_masterB = matrix.relative_to(slaveBoard_t, masterBoard_t)
                slaveB_wrt_slaveCam = np.linalg.inv(np.array(self.all_handEye[masterCam][g]['slaveCam_wrto_slaveB']))
                slaveCam_wrt_slaveB = find_projection(slaveCam_idx, slaveBoard_idx, image_list, slaveCam_matrix,
                                                      slaveCam_dist)
                slaveB_wrt_slaveCam1 = np.linalg.inv(slaveCam_wrt_slaveB)
                ZB = matrix.transform(slaveCam_wrt_masterCam, masterCam_wrt_masterB)
                estimated_slaveB_slaveCam = np.linalg.inv(matrix.transform(ZB, np.linalg.inv(slaveB_wrt_masterB)))
                error = rtvec.from_matrix(estimated_slaveB_slaveCam) - rtvec.from_matrix(slaveB_wrt_slaveCam)
                err.extend(error.reshape((1,-1)).tolist()[0])
            return err
        pass

        cluster_error = {}
        self.cam_pose2['camera_pose'] = {}
        for idxM, master_cam in enumerate(self.workspace.names.camera):
            self.cam_pose2['camera_pose'][master_cam] = {}
            self.cam_pose2['camera_pose'][master_cam][master_cam] = np.eye(4).tolist()
            cluster_error[master_cam] = {}
            # cameraIntrinsic + camera_poses + boards
            param_vec = self.campose_param[master_cam] + self.board_param
            for idxS, slave_cam in enumerate(self.workspace.names.camera):
                error = []
                # param_vec = []
                if master_cam != slave_cam:

                    slaveCam_wrt_masterCam = rtvec.from_matrix(np.array(self.cam_pose[master_cam][slave_cam]['extrinsic'])).tolist()
                    # param_vec.extend(slaveCam_wrt_masterCam)
                    groups = self.cam_pose[master_cam][slave_cam]['group']
                    err1 = evaluation(np.array(param_vec), master_cam, groups)
                    point_errors1, rms1, min_group1 = reprojection_error(np.array(param_vec), master_cam, groups)
                    groups = [min_group1]
                    res = least_squares(evaluation, param_vec,
                                        verbose=2, x_scale='jac', f_scale=1.0, ftol=1e-4, max_nfev=100,
                                        method='trf', loss='linear', args=(master_cam, groups))
                    err2 = evaluation(res.x, master_cam, groups)

                    point_errors2, rms2, min_group2 = reprojection_error(res.x, master_cam, groups)
                    groups = [min_group2]
                    inliers = np.ones(len(point_errors2))
                    for i in range(5):
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
                    point_errors, rms, _ = reprojection_error(res.x, master_cam, groups)
                    self.cam_pose2['camera_pose'][master_cam][slave_cam] = rtvec.to_matrix(res.x[0:6]).tolist()
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

        def fun(param_vec):
            slaveCam_wrt_masterCam = rtvec.to_matrix(param_vec[10:16])
            slaveB_wrt_masterB = rtvec.to_matrix(param_vec[16:22])
            ZB = matrix.transform(slaveCam_wrt_masterCam, masterCam_wrt_masterB)
            estimated_slaveB_slaveCam = np.linalg.inv(matrix.transform(ZB, np.linalg.inv(slaveB_wrt_masterB)))
            return estimated_slaveB_slaveCam

        def evaluation(param_vec):
            estimated_slaveB_slaveCam = fun(param_vec)
            camMatrix, camDist = decode_intrinsic(param_vec[0:10])
            point_errors, reprojection_error = self.reprojectionError_calculation(slave_cam, boardS,
                                                                                  estimated_slaveB_slaveCam, image_list,
                                                                                  camMatrix=camMatrix, camDist=camDist)
            errors = np.array([point_errors[i] for i, v in enumerate(inliers) if v == True ])
            return errors

        def calculate_meanError(param_vec):
            estimated_slaveB_slaveCam = fun(param_vec)
            camMatrix, camDist = decode_intrinsic(param_vec[0:10])
            point_errors, reprojection_error = self.reprojectionError_calculation(slave_cam, boardS,
                                                            estimated_slaveB_slaveCam, image_list, camMatrix=camMatrix, camDist=camDist)
            mean_error = reprojection_error
            return point_errors, mean_error


        ### new
        slaveCam_vec = rtvec.from_matrix(init_slaveCam_wrt_masterCam)
        slaveB_vec = rtvec.from_matrix(init_slaveB_wrt_masterB)
        intrinsic = np.array(self.camintrinsic_param[slave_cam*10: slave_cam*10+10])
        param_vec = np.concatenate((intrinsic, slaveCam_vec, slaveB_vec))
        point_errors, _ = calculate_meanError(param_vec)
        inliers = np.ones(len(point_errors))

        if num_adjustments ==0:
            return 0
        for i in range(num_adjustments):
            info(f"Adjust_outliers {i}:")
            res = least_squares(evaluation, param_vec,
                                verbose=2, x_scale='jac', f_scale=1.0, ftol=1e-4, max_nfev=100,
                                method='trf', loss='linear')
            point_errors, _ = calculate_meanError(res.x)
            threshold = np.quantile(point_errors, 0.75)
            inliers = np.array(point_errors < threshold).flatten()

        ### new
        _, mean_error = calculate_meanError(res.x)
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
            imagePoints = self.workspace.point_table.points[slaveCam][images.index(image_list[i])][slaveBoard][point_ids]
            undistorted = self.workspace.cameras[slaveCam].undistort_points(imagePoints).astype('float32')
            # imagePoints = self.workspace.detected_points[slaveCam][images.index(image_list[i])][slaveBoard]['corners']
            # point_ids = self.workspace.detected_points[slaveCam][images.index(image_list[i])][slaveBoard]['ids']
            repo_error = self.workspace.pose_table.reprojection_error[slaveCam][images.index(image_list[i])][slaveBoard]
            # if repo_error < 0.5:
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

    def master_slave_pose(self, master_cam, master_board, slave_cam, slave_board):
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

        for idx, img in enumerate(self.workspace.names.image):
            master_valid = self.workspace.pose_table.valid[master_cam][idx][master_board]
            slave_valid = self.workspace.pose_table.valid[slave_cam][idx][slave_board]
            if master_valid and slave_valid:
                master_pose = np.linalg.inv(self.workspace.pose_table.poses[master_cam][idx][master_board])
                master_R, master_t = matrix.split(master_pose)
                slave_pose = np.linalg.inv(self.workspace.pose_table.poses[slave_cam][idx][slave_board])
                slave_R, slave_t = matrix.split(slave_pose)
                masterR_list.append(master_R)
                masterT_list.append(master_t)
                slaveR_list.append(slave_R)
                slaveT_list.append(slave_t)
                image_list.append(img)

        return np.array(masterR_list), np.array(masterT_list), np.array(slaveR_list), np.array(slaveT_list), image_list

    def export_handEye_Camera(self):
        filename = os.path.join(base_path, "handEyeCamera.json")
        json_object = json.dumps((self.all_handEye), indent=4)
        # Writing to sample.json
        with open(filename, "w") as outfile:
            outfile.write(json_object)
        pass

    def export_campose2(self):
        filename = os.path.join(base_path, "campose2.json")
        json_object = json.dumps((self.cam_pose2), indent=4)
        # Writing to sample.json
        with open(filename, "w") as outfile:
            outfile.write(json_object)
        pass

    def export_camera_board_param(self):
        param = {}
        param['camera_pose'] = self.cam_pose
        param['board_pose'] = self.board_pose
        filename = os.path.join(base_path, "CameraBoard_param.json")
        json_object = json.dumps((param), indent=4)
        # Writing to sample.json
        with open(filename, "w") as outfile:
            outfile.write(json_object)
        pass

def main1(base_path, cam, board):
    '''
    Input->  base_path : It takes folder containing only 1 camera, board.yaml, calibration.json, gripper_pose.json
             cam : int (choice 0)
             board : int (choice 0,1,2,3,4)
    Output->  Create handEyeGripper.json file containing relation between that camera and one board
    '''
    h = handEye(base_path)
    h.initiate_workspace()
    h.set_gripper_pose()
    handEye_struct = h.handEye_gripper(camera=cam, board=board)
    h.export_handEyeGripper(handEye_struct)
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

def main4(base_path, limit_images, num_adjustments):

    h = handEye(base_path)
    h.initiate_workspace()
    h.calc_camPose_param(limit_images, num_adjustments)
    h.check_cluster_reprojectionerr()
    h.export_handEye_Camera()
    h.export_camera_board_param()
    # h.export_campose2()
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
    # main1(base_path, cam=0, board=3)
    # main3(base_path, limit_images=10, num_adjustments=2)
    # main4(base_path, limit_images=10, num_adjustments=0)
    # main5(base_path, limit_images=10, num_adjustments=1)
    main6(base_path, limit_images=10, num_adjustments=1)
    pass