import numpy as np

import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
from src.multical.tables import *
from structs.struct import transpose_structs, invert_keys
from structs.numpy import shape_info, struct, Table, shape

import copy
import json

path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\handEye_gripper/08320220/08320220"
poseJsonPath = "D:\MY_DRIVE_N\Masters_thesis\Dataset\handEye_gripper/08320220/08320220/stream_220.json"
board_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\handEye_gripper/08320220/08320220/boards.yaml"
intrinsic_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\handEye_gripper/08320220/08320220/calibration.json"

class handEye():
    def __init__(self, datasetPath, boardPath, intrinsic_path, poseJsonPath):
        self.datasetPath = datasetPath
        self.boardPath = boardPath
        self.poseJsonPath = poseJsonPath
        self.intrinsicPath = intrinsic_path
        self.workspace = None
        self.gripper_pose = {}

    def initiate_workspace(self):
        pathO = args.PathOpts(image_path=self.datasetPath)
        cam = args.CameraOpts(motion_model="calibrate_board",
                              calibration=self.intrinsicPath)
        runt = args.RuntimeOpts()
        opt = args.OptimizerOpts(outlier_threshold=1.2, fix_intrinsic=True)
        c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
        self.workspace = c.execute_new()
        self.workspace.pose_table = make_pose_table(self.workspace.point_table, self.workspace.boards,
                                                    self.workspace.cameras)

    def handEye_gripper(self, camera, board):
        board_cam_pose = self.workspace.pose_table._index_select(camera, axis=0)._index_select(board, axis=1)

        R_cam2world_list = []
        t_cam2world_list = []
        R_base2gripper_list = []
        t_base2gripper_list = []
        image_name = self.workspace.names.image
        for img in image_name:
            p = image_name.index(img)
            grip_pose = img[:-4][1:]
            camBoard_valid = board_cam_pose.valid[p]
            if camBoard_valid:
                board_to_cam = board_cam_pose.poses[p]
                R_cam2world, t_cam2world = matrix.split(np.linalg.inv(board_to_cam))
                R_base2gripper, t_base2gripper = matrix.split((self.gripper_pose[grip_pose]))
                R_cam2world_list.append(R_cam2world)
                t_cam2world_list.append(t_cam2world)
                R_base2gripper_list.append(R_base2gripper)
                t_base2gripper_list.append(t_base2gripper)

        # board_wrt_boardM, slave_wrt_master, world_wrt_camera, \
        #                     base_wrt_gripper, err, err2
        # base_wrt_world, gripper_wrt_camera, world_wrt_camera, base_wrt_gripper
        # base_wrt_cam, gripper_wrt_world, camera_wrt_world, base_wrt_gripper
        base_wrt_cam, gripper_wrt_world, camera_wrt_world, base_wrt_gripper, err, err2 = hand_eye.hand_eye_robot_world(np.array(R_cam2world_list),
                                            np.array(t_cam2world_list), np.array(R_base2gripper_list), np.array(t_base2gripper_list))
        b = np.linalg.inv(base_wrt_cam)
        g = np.linalg.inv(gripper_wrt_world)

        base_cam_r, base_cam_t, gripper_world_r, gripper_world_t = self.hand_eye_robot_world(np.array(R_cam2world_list),
                                            np.array(t_cam2world_list), np.array(R_base2gripper_list), np.array(t_base2gripper_list))
        pass

    def hand_eye_robot_world(self, cam_world_R, cam_world_t, base_gripper_R, base_gripper_t):
        base_cam_r, base_cam_t, gripper_world_r, gripper_world_t = \
            cv2.calibrateRobotWorldHandEye(cam_world_R, cam_world_t, base_gripper_R, base_gripper_t, method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH)

        base_wrt_cam = matrix.join(base_cam_r, base_cam_t.reshape(-1))
        gripper_wrt_world = matrix.join(gripper_world_r, gripper_world_t.reshape(-1))
        camera_wrt_world = matrix.join(cam_world_R, cam_world_t)
        base_wrt_gripper = matrix.join(base_gripper_R, base_gripper_t)

        err = matrix.transform(base_wrt_cam, camera_wrt_world) - matrix.transform(base_wrt_gripper, gripper_wrt_world)
        ZB = matrix.transform(base_wrt_cam, camera_wrt_world)
        error2 = base_wrt_gripper - matrix.transform(np.linalg.inv(gripper_wrt_world), ZB)

        return base_cam_r, base_cam_t, gripper_world_r, gripper_world_t

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


    def estimate_camera_board_poses(self):
        '''
        for Master_cam to Slave_cam
        :return:
        '''
        num_cameras = len(self.workspace.names.camera)
        handEye_dict = {}
        for cam in range(0, num_cameras):
            handEye_dict[cam] = self.handEye_table(master_cam=cam)

    def handEye_table(self, master_cam=0):
        '''
        handEye for Master_cam to Slave_cam
        :param master_cam:
        :return:
        '''
        num_cameras = len(self.workspace.names.camera)
        num_boards = len(self.workspace.names.board)

        handEye_dict = {}
        serial = 0
        handEye_dict['slave_wrt_master'] = {}
        handEye_dict['board_wrt_boardM'] = {}
        master_boards = [b for b in range(0, num_boards) if self.workspace.pose_table.valid[master_cam][:, b].sum() > 20]
        for slave_cam in range(0, num_cameras):
            slave_boards = [b for b in range(0, num_boards) if self.workspace.pose_table.valid[slave_cam][:, b].sum() > 20]

            for boardM in master_boards:
                for boardS in slave_boards:
                    masterR, masterT, slaveR, slaveT, image_list = self.master_slave_pose(master_cam, boardM,
                                                                                     slave_cam, boardS)
                    board_wrt_boardM, slave_wrt_master, world_wrt_camera, \
                    base_wrt_gripper, err, err2 = hand_eye.hand_eye_robot_world(masterR, masterT, slaveR, slaveT)
                    reprojection_error = self.reprojectionError_calculation(board_wrt_boardM, slave_wrt_master, world_wrt_camera,
                                                                            base_wrt_gripper, slave_cam, boardS, image_list)
                    handEye_dict[serial] = struct(master_cam=master_cam, master_board=boardM, slave_cam = slave_cam, slave_board=boardS,
                                                  boardS_wrto_boardM=board_wrt_boardM, slaveCam_wrto_masterCam=slave_wrt_master, reprojection_error=reprojection_error)
                    serial += 1

        return handEye_dict

    def reprojectionError_calculation(self, board_wrt_boardM, slave_wrt_master, world_wrt_camera, base_wrt_gripper, slave_cam, boardS, image_list):
        '''
        for Master_cam to Slave_cam
        '''
        error_dict = {}
        T1 = matrix.transform(base_wrt_gripper, board_wrt_boardM)
        T2 = matrix.transform(np.linalg.inv(slave_wrt_master), T1)
        cameraMatrix = self.workspace.cameras[slave_cam].intrinsic
        distortion = self.workspace.cameras[slave_cam].dist
        for i in range(0, T2.shape[0]):
            imagePoints = self.workspace.detected_points[slave_cam][image_list[i]][boardS]['corners']
            point_ids = self.workspace.detected_points[slave_cam][image_list[i]][boardS]['ids']
            objectPoints = np.array([self.workspace.boards[boardS].adjusted_points[i] for i in point_ids])
            rmatrix = T2[i][0:3, 0:3]
            tmatrix = T2[i][0:3, 3]
            rvec = (R.from_matrix([rmatrix]).as_rotvec())
            tvec = (tmatrix.T)
            imP, _ = cv2.projectPoints(np.copy(objectPoints), rvec, tvec, cameraMatrix, distortion)
            error = imagePoints - imP.reshape([-1, 2])
            error_dict[i] = error
        return error_dict

    def master_slave_pose(self, master_cam, master_board, slave_cam, slave_board):
        num_images = len(self.workspace.names.image)
        '''
        For Master_cam to Slave_cam
        '''
        masterR_list = []
        masterT_list = []
        slaveR_list = []
        slaveT_list = []
        image_list = []

        for img in range(0, num_images):
            master_valid = self.workspace.pose_table.valid[master_cam][img][master_board]
            slave_valid = self.workspace.pose_table.valid[slave_cam][img][slave_board]
            if master_valid and slave_valid:
                master_pose = self.workspace.pose_table.poses[master_cam][img][master_board]
                master_R, master_t = matrix.split(master_pose)
                slave_pose = self.workspace.pose_table.poses[slave_cam][img][slave_board]
                slave_R, slave_t = matrix.split(slave_pose)
                masterR_list.append(master_R)
                masterT_list.append(master_t)
                slaveR_list.append(slave_R)
                slaveT_list.append(slave_t)
                image_list.append(img)

        return np.array(masterR_list), np.array(masterT_list), np.array(slaveR_list), np.array(slaveT_list), image_list

if __name__ == '__main__':
    h = handEye(path, board_path, intrinsic_path, poseJsonPath)
    h.initiate_workspace()
    h.set_gripper_pose()
    h.handEye_gripper(camera=0, board=1)
    # h.estimate_camera_board_poses()
    pass