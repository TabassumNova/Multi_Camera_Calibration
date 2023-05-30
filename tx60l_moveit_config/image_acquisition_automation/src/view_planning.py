import sys
import roslib
import xlsxwriter

import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import glob

import tf
# import camStreamer.CamStreamer
from .camStreamer import CamStreamer
from .helpers import *
from .TIS_image_acquisition import start_tis_image_acquisition
from .aravis_image_acquisition import *
from .data_robot_mover2 import *

import multical.app.boards as boards

output_path = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/view_plan/'
board_yaml = "/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/V11/V3/input/data/boards.yaml"


# manually place one board very near to one camera
# then start the program
def view_Plan(box_attacher):
    get_intrinsic_poses()
    wpose = box_attacher.move_group.get_current_pose().pose

    circular_path(box_attacher, wpose)

    # pose = 1
    # # analyze_camera_images(output_path, pose)
    # # get_intrinsic_poses()
    # # calculate the slope to robot base
    # slope_base = wpose.position.y/wpose.position.x
    # if wpose.position.x<0:
    #     start = wpose.position.x
    #     end = 0.0
    #     step = 0.01
    #     plan_Line(box_attacher, start, end, step, slope_base, pose, output_path)
    # else:
    #     start = wpose.position.x
    #     end = 0.0
    #     step = -0.01
    #     plan_Line(box_attacher, start, end, step, slope_base)
    # pass

def circular_path(box_attacher, wpose, radious= 0.05, step=10):
    theta = np.linspace(0, 2 * np.pi, step)
    # the radius of the circle
    r = np.sqrt(radious)
    # compute x, y
    x = r * np.cos(theta) + wpose.position.x
    y = r * np.sin(theta) + wpose.position.y

    pose_num = 1
    for path in range(0, len(x)):
        # for plan_x in range(start, end, step):
        pose = box_attacher.move_group.get_current_pose().pose
        pose.position.x = x[path]
        pose.position.y = y[path]
        print('Pose: ', pose)
        plan = box_attacher.move_group.go(pose, wait=True)
        box_attacher.move_group.stop()
        camera_serial = arv_get_image(output_path, pose_num)
        pose_num += 1
        pose_num = change_orientation(box_attacher, pose_num)

def change_orientation(box_attacher, pose):
    initial_pose = get_pose(box_attacher, euler=True)
    pose_list = get_calib_poses(box_attacher, initial_pose)
    for i in range(len(pose_list)):
        print('Pose: ', pose)
        motion_successful = move_robot(box_attacher, pose_list[i])
        camera_serial = arv_get_image(output_path, pose)
        pose += 1
    return pose

def detect_board(image_path):
    b = boards.Boards(boards=board_yaml, detect=image_path, pixels_mm=10, show_image=False)
    detections = b.execute()
    return detections

def analyze_camera_images(box_attacher, output_path, pose):
    camera_serial = arv_get_image(output_path, pose)
    camera_board = {}
    camera_views = {}
    for cam in range(len(camera_serial)):
        view = 1
        camera_board[cam] = {}
        image_path = output_path + camera_serial[cam] + '/p' + str(pose) + '.png'
        detections = detect_board(image_path)
        max = 0
        for d in detections:
            ## choose most detected board of that camera
            if d.ids.size > max:
                board = d
                camera_board[cam]['board'] = board
                camera_board[cam]['detected_points'] = d.ids.size

        if camera_board[cam]['detected_points'] > 0:
            pose = box_attacher.move_group.get_current_pose().pose
            camera_views[cam][view] = pose
            view += 1


def get_intrinsic_poses(box_attacher):
    # joint infos
    common_focus = [-61, -19, 117, 112, 5, -247]
    cameras = {}
    cameras['cam_218'] = [117, -55, -14, 186, 25, -143]
    cameras['cams_220'] = [-31, -5, 100, 102 , -19, -236]
    cameras['cam_113'] = [113, -52, -24, -108, 69, 150]
    cameras['cam_222'] = [85, -16, -82, -88, 59, 113]

    plan = box_attacher.move_robot_joints(np.array(common_focus))
    common_focus_pose = box_attacher.move_group.get_current_pose().pose
    for keys in cameras:
        pose_num = 1
        plan1 = box_attacher.move_robot_joints(np.array(cameras[keys]))
        camera_serial = arv_get_image(output_path, pose_num)
        pose_num += 1
        pose_num = change_orientation(box_attacher, pose_num)
        plan2 = box_attacher.move_robot_joints(np.array(cameras[keys]))
        cam_pose = box_attacher.move_group.get_current_pose().pose
        plan_line(box_attacher, cam_pose, common_focus_pose, step=2)
        camera_serial = arv_get_image(output_path, pose_num)
        pose_num += 1
        pose_num = change_orientation(box_attacher, pose_num)

def plan_line(box_attacher, start, end, step):
    start_x = start.position.x
    end_x = end.position.x
    start_y = start.position.y
    end_y = end.position.y
    slope = (end_y-start_y)/(end_x-start_x)
    poses = np.linspace(start_x, end_x, step)
    for plan_x in poses:
        pose = box_attacher.move_group.get_current_pose().pose
        pose.position.x = plan_x
        pose.position.y = slope * plan_x
        plan = box_attacher.move_group.go(pose, wait=True)
        box_attacher.move_group.stop()

def plan_Line_old(box_attacher, start, end, step, slope, pose_num, path):
    poses = np.linspace(start, end, 10)
    for plan_x in poses:
    # for plan_x in range(start, end, step):
        pose = box_attacher.move_group.get_current_pose().pose
        pose.position.x = plan_x
        pose.position.y = slope*plan_x
        plan = box_attacher.move_group.go(pose, wait=True)
        box_attacher.move_group.stop()
        analyze_camera_images(box_attacher, path, pose_num)
        pose_num += 1
