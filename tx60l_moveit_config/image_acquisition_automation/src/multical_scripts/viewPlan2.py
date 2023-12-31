import numpy as np

import random
import src.multical.app.boards as boards
import os

from src.aravis_image_acquisition import *

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from src.data_robot_mover2 import *
from src.aravis_image_acquisition import *
from src.multical.workspace import Workspace
from src.multical.config.runtime import *
import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
from src.multical.tables import *

import copy
import json

# path = 'D:\MY_DRIVE_N\Masters_thesis\Dataset/train/'
# json_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset/train\poses_viewPlan2.json"
# board_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset/train/boards.yaml"
# intrinsic_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset/train/all_camera_intrinsic.json"

class viewPlan2():
    def __init__(self, box_attacher, datasetPath, boardPath, intrinsic_path, poseJsonPath):
    # def __init__(self, datasetPath, boardPath, intrinsic_path, poseJsonPath):
        self.box_attacher = box_attacher
        self.datasetPath = datasetPath
        self.boardPath = boardPath
        self.poseJsonPath = poseJsonPath
        self.intrinsicPath = intrinsic_path
        self.workspace = None
        self.json_dict = {}

        enter = input("Hit ENTER if you want to start planning: ")
        self.reset_position()
        self.initial_pose = get_pose(self.box_attacher, euler=True)

        self.pose = 1
        self.detection_dict = {}
        self.camera_serial = arv_get_image(self.datasetPath, self.pose)
        self.initiate_workspace()
        # self.align_board_perCamera()
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

    def align_board_perCamera(self):
        for cam in range(0, self.workspace.sizes.camera):
            boards = self.select_valid_boards(cam)
            best_board, view_angles, reprojectionError= self.select_best_viewed_board(cam, boards)
            if best_board != None:
                self.moveBoard_iteratively(cam, best_board)
            self.reset_position()
        pass

    def moveBoard_iteratively(self, cam, board):
        # First adjust angles
        angles = self.get_view_angles(cam, self.pose, board)
        print("Camera {}: Start adjusting Angles for board {} from pose {}".format(self.camera_serial[cam], board, self.pose))
        while (abs(angles[0]) > 2) & (abs(angles[1]) > 2):
            self.adjust_angles(board, angles)
            self.initiate_workspace()
            angles = self.get_view_angles(cam, self.pose, board)
        print("Camera {}: Successfully adjusted Angles for board {} from pose {}".format(self.camera_serial[cam], board, self.pose))

        # Now adjust position
        area, center_x, center_y = self.check_coverage_area(cam, board)
        image_width, image_height = 5472, 3648
        print("Camera {}: Start adjusting Angles for board {} from pose {}".format(self.camera_serial[cam], board, self.pose+1))
        while (abs(center_x - image_width)>200) & (abs(center_y - image_height)>200):
            self.adjust_position(board, [center_x, center_y])
            area, center_x, center_y = self.check_coverage_area(cam, board)
        print("Camera {}: Successfully adjusted Angles for board {} from pose {}".format(self.camera_serial[cam], board,
                                                                                         self.pose))
        pass

    def adjust_angles(self, board, angles):
        if board == 0:
            pass
        elif board == 1:
            pass
        elif board == 2:
            transformation = get_orientation(self.box_attacher, self.initial_pose, [angles[1]*(-1), 0, 0])
            motion_successful = move_robot(self.box_attacher, transformation)
            current_pose = get_pose(self.box_attacher, euler=True)
            transformation = get_orientation(self.box_attacher, self.initial_pose, [0, angles[0]*(-1), 0])
            motion_successful = move_robot(self.box_attacher, transformation)
            self.pose += 1
            self.camera_serial = arv_get_image(self.datasetPath, self.pose)
            pass
        elif board == 3:
            pass
        elif board == 4:
            pass

    def adjust_position(self, board, position):
        if board == 0:
            pass
        elif board == 1:
            pass
        elif board == 2:
            position_change = np.array((position[1]*(-1), position[0]*(-1), 0))
            translation = get_position(self.box_attacher, self.initial_pose, position_change)
            motion_successful = move_robot(self.box_attacher, translation)
            self.pose += 1
            self.camera_serial = arv_get_image(self.datasetPath, self.pose)
            pass
        elif board == 3:
            pass
        elif board == 4:
            pass

    def checkDataset(self):
        self.checkPose()
        pass

    def checkPose(self):
        for cam in range(0, self.workspace.sizes.camera):
            boards = self.select_valid_boards(cam)
            best_board, view_angles, reprojectionError= self.select_best_viewed_board(cam, boards, pose=1)
            # area = self.check_coverage_area(cam, best_board)
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
        center_x, center_y = (x_max - x_min)/2, (y_max - y_min)/2
        return area, center_x, center_y

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
            x = [10, 20, 30, 40, -10, -20, -30, -40]
            y = [10, 20, 30, 40, -10, -20, -30, -40]
            z = [10, 20, 30, 40, -10, -20, -30, -40]
            for i in x:
                transformation = get_orientation(self.box_attacher, self.initial_pose, [i, i, 0])
                translation_list.append(transformation)
            for i in x:
                transformation = get_orientation(self.box_attacher, self.initial_pose, [i, 0, 0])
                translation_list.append(transformation)
            for i in y:
                transformation = get_orientation(self.box_attacher, self.initial_pose, [0, i, 0])
                translation_list.append(transformation)
            for i in z:
                transformation = get_orientation(self.box_attacher, self.initial_pose, [0, 0, i])
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
            self.write_json()
            motion_successful = move_robot(self.box_attacher, translation)
            self.pose += 1
            self.camera_serial = arv_get_image(self.datasetPath, self.pose)

    def write_json(self):
        self.json_dict[self.pose] = {}
        current_pose = self.box_attacher.move_group.get_current_pose().pose.position
        current_orientation = self.box_attacher.move_group.get_current_pose().pose.orientation
        self.json_dict[self.pose]['position (x,y,z)'] = [str(current_pose.x), str(current_pose.y), str(current_pose.z)]
        self.json_dict[self.pose]['orintation (w,x,y,z)'] = [str(current_orientation.w), str(current_orientation.x), str(current_orientation.y), str(current_orientation.z)]
        self.json_dict[self.pose]['joint_goal'] = [str(a) for a in self.box_attacher.move_group.get_current_joint_values()]

        # Serializing json
        json_object = json.dumps(self.json_dict, indent=4)
        # Writing to sample.json
        with open(self.poseJsonPath, "w") as outfile:
            outfile.write(json_object)

    def select_valid_boards(self, cam):
        return [idx for idx, value in enumerate(self.workspace.pose_table.valid[cam][self.pose-1]) if value==True]

    def select_best_viewed_board(self, cam, boards, pose=self.pose):
        view_angle = 500
        best_board = None
        for b in boards:
            angles = self.get_view_angles(cam, pose, b)
            reprojectionError = self.workspace.pose_table.reprojection_error[cam][pose-1][b]
            total_angle = abs(angles[0]) + abs(angles[1])
            if total_angle < view_angle:
                best_board = b
                view_angle = total_angle
        return best_board, angles, reprojectionError

    def get_view_angles(self, cam, pose, board):
        return self.workspace.pose_table.view_angles[cam][pose-1][board]



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

# if __name__ == '__main__':
#     v = viewPlan2(box_attacher, path, board_path, intrinsic_path, json_path)