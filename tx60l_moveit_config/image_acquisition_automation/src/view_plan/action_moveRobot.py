import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from src.data_robot_mover2 import *

def moveRobot(box_attacher, action):
    # action_encode = [-1, 0, 1]
    translation_step = 0.05
    rotation_step = 10

    initial_pose = get_pose(box_attacher, euler=True)
    position_change_x = (action['move_x']-1) * translation_step
    position_change_y = (action['move_y']-1) * translation_step
    position_change_z = (action['move_z']-1) * translation_step

    position_change = np.array((position_change_x, position_change_y, position_change_z))
    translation = get_position(box_attacher, initial_pose, position_change)

    motion_successful = move_robot(box_attacher, translation)

    initial_pose = get_pose(box_attacher, euler=True)
    rotation_degree = []
    rotation_degree.append((action['rotate_x']-1)*rotation_step)
    rotation_degree.append((action['rotate_y']-1) * rotation_step)
    rotation_degree.append((action['rotate_z']-1) * rotation_step)
    transformation = get_orientation(box_attacher, initial_pose, rotation_degree)
    motion_successful = move_robot(box_attacher, transformation)
