import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from src.data_robot_mover2 import *

def moveRobot(box_attacher, action):
    translation_step = 0.1
    rotation_step = 10

    wpose = box_attacher.move_group.get_current_pose().pose
    wpose.position.x = wpose.position.x + action['move_x'] * translation_step
    wpose.position.y = wpose.position.y + action['move_y'] * translation_step
    wpose.position.z = wpose.position.z + action['move_z'] * translation_step
    plan = box_attacher.move_group.go(wait=True)
    box_attacher.move_group.stop()

    initial_pose = get_pose(box_attacher, euler=True)
    rotation_degree = []
    rotation_degree[0] = action['rotate_x']*rotation_step
    rotation_degree[1] = action['rotate_y'] * rotation_step
    rotation_degree[2] = action['rotate_z'] * rotation_step
    transformation = get_orientation(box_attacher, initial_pose, rotation_degree)
    motion_successful = move_robot(box_attacher, transformation)
