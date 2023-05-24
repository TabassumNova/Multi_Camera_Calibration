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
    wpose = box_attacher.move_group.get_current_pose().pose
    pose = 1
    # analyze_camera_images(output_path, pose)
    # get_intrinsic_poses()
    # calculate the slope to robot base
    slope_base = wpose.position.y/wpose.position.x
    if wpose.position.x<0:
        start = wpose.position.x
        end = 0.0
        step = 0.01
        plan_Line(box_attacher, start, end, step, slope_base)
    else:
        start = wpose.position.x
        end = 0.0
        step = -0.01
        plan_Line(box_attacher, start, end, step, slope_base)
    pass

def detect_board(image_path):
    b = boards.Boards(boards=board_yaml, detect=image_path, pixels_mm=10)
    b.execute()
    pass

def analyze_camera_images(output_path, pose):
    camera_serial = arv_get_image(output_path, pose)
    for cam in range(len(camera_serial)):
        image_path = output_path + camera_serial[cam] + '/p' + str(pose) + '.png'
        # image = glob.glob(path)[0]
        detect_board(image_path)

    pass

def get_intrinsic_poses():
    pass


def plan_Line(box_attacher, start, end, step, slope):
    poses = np.linspace(start, end, 10)
    for plan_x in poses:
    # for plan_x in range(start, end, step):
        pose = box_attacher.move_group.get_current_pose().pose
        pose.position.x = plan_x
        pose.position.y = slope*plan_x
        plan = box_attacher.move_group.go(wait=True)
        box_attacher.move_group.stop()
        analyze_camera_images()
