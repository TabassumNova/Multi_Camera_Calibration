import numpy as np

import random
import src.multical.app.boards as boards
import os

# from src.aravis_image_acquisition import *

# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg

# from src.data_robot_mover2 import *
# from src.aravis_image_acquisition import *
from src.multical.workspace import Workspace
from src.multical.config.runtime import *
import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
from src.multical.tables import *

import copy
import json

path = 'D:\MY_DRIVE_N\Masters_thesis\Dataset/train/'
json_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset/train\poses_geo.json"
board_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset/train/boards.yaml"
intrinsic_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset/train/all_camera_intrinsic.json"

class viewPlan2():
    # def __init__(self, box_attacher, datasetPath, boardPath, intrinsic_path, poseJsonPath):
    def __init__(self, datasetPath, boardPath, intrinsic_path, poseJsonPath):
        # self.box_attacher = box_attacher
        self.datasetPath = datasetPath
        self.boardPath = boardPath
        self.poseJsonPath = poseJsonPath
        self.intrinsicPath = intrinsic_path
        self.workspace = None
        # self.initial_pose = get_pose(self.box_attacher, euler=True)

        enter = input("Hit ENTER if you want to start planning: ")
        # self.reset_position()

        self.pose = 1
        self.detection_dict = {}
        # self.camera_serial = arv_get_image(path, self.pose)
        self.initiate_workspace()
        self.checkDataset()

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
    def reset_position(self):
        common_focus = [-61, -19, 117, 112, 5, -247]
        plan = self.box_attacher.move_robot_joints(np.array(common_focus))

    def checkDataset(self):
        self.checkPose()
        pass

    def checkPose(self):
        for cam in range(0, self.workspace.sizes.camera):
            boards = self.select_valid_boards(cam)
            best_board, view_angle, reprojectionError= self.select_best_viewed_board(cam, boards)
            self.check_coverage_area(cam, best_board)
            if best_board != None:
                self.moveBoard(best_board)

    def check_coverage_area(self, cam, board):
        points = self.workspace.point_table.points[cam][self.pose][board]
        valid = self.workspace.point_table.points[cam][self.pose][board]
        x_points = np.array([points._index_select(i, axis=0) for i in range(0, points.shape[0]) if valid[i] == True])
        y_points = np.array([points._index_select(i, axis=1) for i in range(0, points.shape[0]) if valid[i] == True])
        x_max, x_min = x_points.max(), x_points.min()
        y_max, y_min = y_points.max(), y_points.min()
        area = (x_max - x_min)*(y_max - y_min)
        pass

    def moveBoard(self, board):
        '''
        The movement of end effector depends on Board Configuration.
        In our case,
        Board_0 -> +x
        Board_1 -> +y
        Board_2 -> +z
        Board_3 -> -y
        Board_4 -> -x
        '''

        rotation_step = 10
        if board == 0:
            # Move around -x direction. Don't change x parameter in end effector
            self.moveTranslation(axis='x')
            self.moveRotation(axis='x')
            pass
        elif board == 1:
            # Move around +y direction. Don't change y parameter in end effector
            self.moveTranslation(axis='y')
            self.moveRotation(axis='y')
            pass
        elif board == 2:
            # Move around +z direction. Don't change z parameter in end effector
            self.moveTranslation(axis='z')
            self.moveRotation(axis='z')
            pass
        elif board == 3:
            # Move around -y direction. Don't change y parameter in end effector
            self.moveTranslation(axis='y')
            self.moveRotation(axis='y')
            pass
        elif board == 4:
            # Move around +x direction. Don't change x parameter in end effector
            self.moveTranslation(axis='x')
            self.moveRotation(axis='x')
            pass
        pass

    def moveRotation(self, axis):
        if axis == 'x':
            translation_list = []
            y = [10, 20, -10, -20]
            z = [10, 20, -10, -20]
            for i in y:
                transformation = get_orientation(self.box_attacher, self.initial_pose, [0, i, 0])
                translation_list.append(transformation)
            for i in z:
                transformation = get_orientation(self.box_attacher, self.initial_pose, [0, 0, i])
                translation_list.append(transformation)
            self.moveRobot(translation_list)
            pass
        elif axis == 'y':
            translation_list = []
            x = [10, 20, -10, -20]
            z = [10, 20, -10, -20]
            for i in x:
                transformation = get_orientation(self.box_attacher, self.initial_pose, [i, 0, 0])
                translation_list.append(transformation)
            for i in z:
                transformation = get_orientation(self.box_attacher, self.initial_pose, [0, 0, i])
                translation_list.append(transformation)
            self.moveRobot(translation_list)
            pass
        elif axis == 'z':
            translation_list = []
            x = [10, 20, -10, -20]
            y = [10, 20, -10, -20]
            for i in x:
                transformation = get_orientation(self.box_attacher, self.initial_pose, [i, 0, 0])
                translation_list.append(transformation)
            for i in y:
                transformation = get_orientation(self.box_attacher, self.initial_pose, [0, i, 0])
                translation_list.append(transformation)
            self.moveRobot(translation_list)
            pass

    def moveTranslation(self, axis):
        if axis == 'x':
            translation_list = []
            x_pos, x_neg = [1, 2, 3], [-1, -2, -3]
            y_pos, y_neg = [1, 2, 3], [-1, -2, -3]
            z_pos, z_neg = [1, 2, 3], [-1, -2, -3]

            translation1 = self.positionChange(axis=[0, 1, 1], list1=x_neg, list2=y_neg, list3=z_neg)
            translation2 = self.positionChange(axis=[0, 1, 1], list1=x_neg, list2=y_neg, list3=z_pos)
            translation3 = self.positionChange(axis=[0, 1, 1], list1=x_pos, list2=y_pos, list3=z_neg)
            translation4 = self.positionChange(axis=[0, 1, 1], list1=x_pos, list2=y_pos, list3=z_pos)
            translation5 = self.positionChange(axis=[1, 0, 0], list1=x_pos, list2=y_neg, list3=z_pos)
            translation6 = self.positionChange(axis=[1, 0, 0], list1=x_neg, list2=y_pos, list3=z_neg)
            translation_list = translation1 + translation2 + translation3 + translation4 + translation5 + translation6
            self.moveRobot(translation_list)
            pass
        elif axis == 'y':
            translation_list = []
            x_pos, x_neg = [1,2,3], [-1,-2,-3]
            y_pos, y_neg = [1,2,3], [-1,-2,-3]
            z_pos, z_neg = [1,2,3], [-1,-2,-3]

            translation1 = self.positionChange(axis=[1, 0, 1], list1=x_neg, list2=y_neg, list3=z_neg)
            translation2 = self.positionChange(axis=[1, 0, 1], list1=x_neg, list2=y_neg, list3=z_pos)
            translation3 = self.positionChange(axis=[1, 0, 1], list1=x_pos, list2=y_neg, list3=z_neg)
            translation4 = self.positionChange(axis=[1, 0, 1], list1=x_pos, list2=y_neg, list3=z_pos)
            translation5 = self.positionChange(axis=[0, 1, 0], list1=x_pos, list2=y_neg, list3=z_pos)
            translation6 = self.positionChange(axis=[0, 1, 0], list1=x_pos, list2=y_pos, list3=z_pos)
            translation_list = translation1 + translation2 + translation3 + translation4 + translation5 + translation6
            self.moveRobot(translation_list)
            pass
        elif axis == 'z':
            translation_list = []
            x_pos, x_neg = [1, 2, 3], [-1, -2, -3]
            y_pos, y_neg = [1, 2, 3], [-1, -2, -3]
            z_pos, z_neg = [1, 2, 3], [-1, -2, -3]

            translation1 = self.positionChange(axis=[1, 1, 0], list1=x_neg, list2=y_neg, list3=z_neg)
            translation2 = self.positionChange(axis=[1, 1, 0], list1=x_neg, list2=y_pos, list3=z_pos)
            translation3 = self.positionChange(axis=[1, 1, 0], list1=x_pos, list2=y_neg, list3=z_neg)
            translation4 = self.positionChange(axis=[1, 1, 0], list1=x_pos, list2=y_pos, list3=z_pos)
            translation5 = self.positionChange(axis=[0, 0, 1], list1=x_pos, list2=y_neg, list3=z_pos)
            translation6 = self.positionChange(axis=[0, 0, 1], list1=x_pos, list2=y_pos, list3=z_neg)
            translation_list = translation1 + translation2 + translation3 + translation4 + translation5 + translation6
            self.moveRobot(translation_list)
            pass

    def positionChange(self, axis, list1, list2, list3, translation_step = 0.05):
        '''
        :param axis: [1, 0, 1] # position change in x and z axis
        :param list1: x_xhange_list
        :param list2: y_change_list
        :param list3: z_change_list
        :param translation_step:
        :return:
        '''
        translation_list = []
        assert len(list1) == len(list2) == len(list3)
        for i in range(0, len(list1)):
            position_change_x = axis[0] * list1[i] * translation_step
            position_change_y = axis[1] * list2[i] * translation_step
            position_change_z = axis[2] * list3[i] * translation_step
            position_change = np.array((position_change_x, position_change_y, position_change_z))
            translation = get_position(self.box_attacher, self.initial_pose, position_change)
            translation_list.append(translation)
        return translation_list

    def moveRobot(self, translation_list):
        for translation in translation_list:
            motion_successful = move_robot(self.box_attacher, translation)
            self.pose += 1
            self.camera_serial = arv_get_image(path, self.pose)


    def select_valid_boards(self, cam):
        return [idx for idx, value in enumerate(self.workspace.pose_table.valid[cam][self.pose-1]) if value==True]

    def select_best_viewed_board(self, cam, boards):
        view_angle = 500
        best_board = None
        for b in boards:
            angles = self.workspace.pose_table.view_angles[cam][self.pose-1][b]
            reprojectionError = self.workspace.pose_table.reprojection_error[cam][self.pose-1][b]._max
            total_angle = abs(angles[0]) + abs(angles[1])
            if total_angle < view_angle:
                best_board = b
                view_angle = total_angle
        return best_board, view_angle, reprojectionError


    def detection(self):
        self.detection_dict[self.pose] = {}
        total_detect = 0
        for subdir, dirs, files in os.walk(self.datasetPath):
            corners = 0
            for camera in dirs:
                detect_img = os.path.join(self.datasetPath, camera, 'p' + str(self.pose) + '.png')
                b = boards.Boards(boards=self.boardPath, detect=detect_img, pixels_mm=10, show_image=False)
                detection = b.execute()
                for d in detection:
                    corners += d.ids.size
                key = str(camera) + "_detection"
                self.detection_dict[self.pose][key] = corners
                total_detect += corners
            self.detection_dict[self.pose]["total_detection"] = total_detect
        return self.detection_dict[self.pose]

class poseQuality():
    def __init__(self, imagePath):
        self.imagePath = imagePath
        self.num_corners = None

if __name__ == '__main__':
    v = viewPlan2(path, board_path, intrinsic_path, json_path)