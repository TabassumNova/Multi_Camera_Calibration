import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from src.data_robot_mover2 import *

def moveRobot(box_attacher, action):
    action_encode = [-1, 0, 1]
    translation_step = 0.05
    rotation_step = 10

    initial_pose = get_pose(box_attacher, euler=True)
    position_change_x = action_encode[action['move_x']] * translation_step
    position_change_y = action_encode[action['move_y']] * translation_step
    position_change_z = action_encode[action['move_z']] * translation_step

    position_change = np.array((position_change_x, position_change_y, position_change_z))
    translation = get_position(box_attacher, initial_pose, position_change)

    motion_successful = move_robot(box_attacher, translation)

    initial_pose = get_pose(box_attacher, euler=True)
    rotation_degree = []
    rotation_degree.append(action_encode[action['rotate_x']]*rotation_step)
    rotation_degree.append(action_encode[action['rotate_y']] * rotation_step)
    rotation_degree.append(action_encode[action['rotate_z']] * rotation_step)
    transformation = get_orientation(box_attacher, initial_pose, rotation_degree)
    motion_successful = move_robot(box_attacher, transformation)
