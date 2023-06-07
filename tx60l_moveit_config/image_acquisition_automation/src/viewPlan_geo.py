import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from src.data_robot_mover2 import *
from src.aravis_image_acquisition import *
import copy
import json

path = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/view_plan/geometrical_method/'
json_path = "/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/view_plan/poses_geo.json"

def viewPlanGeo(box_attacher):
    enter = input("Hit ENTER if you want to start planning: ")
    common_focus = [-61, -19, 117, 112, 5, -247]
    plan = box_attacher.move_robot_joints(np.array(common_focus))

    pose_list = viewPlan_list(box_attacher)
    json_dict = {}
    pose = 1
    if enter == '':
        for i in range(len(pose_list)):
            # Move to first point automatically
            arv_get_image(path, pose)
            pass
            print('Pose: ', pose)
            json_dict[pose] = {}
            current_pose = get_pose(box_attacher, euler=True)
            # current_orientation = box_attacher.move_group.get_current_pose().pose.orientation
            json_dict[pose]['position (x,y,z)'] = [str(current_pose['position'][0]), str(str(current_pose['position'][1])), str(current_pose['position'][2])]
            json_dict[pose]['orintation (x,y,z)'] = [str(current_pose['orientation'][0]), str(current_pose['orientation'][1]), str(current_pose['orientation'][2])]
            json_dict[pose]['joint_goal'] = [str(a) for a in box_attacher.move_group.get_current_joint_values()]

            # Serializing json
            json_object = json.dumps(json_dict, indent=4)
            # Writing to sample.json
            with open(json_path, "w") as outfile:
                outfile.write(json_object)

                motion_successful = move_robot(box_attacher, pose_list[i])
                pose += 1
        
        # # Serializing json
        # json_object = json.dumps(json_dict, indent=4)
        # # Writing to sample.json
        # with open(json_path, "w") as outfile:
        #     outfile.write(json_object)


    pass

def viewPlan_list(box_attacher):
    xBound = np.concatenate([np.arange(0, 0.5, 0.05), np.arange(0, -0.5, -0.05)])
    yBound = np.concatenate([np.arange(0, 0.5, 0.05), np.arange(0, -0.5, -0.05)])
    zBound = np.concatenate([np.arange(0, 0.5, 0.05), np.arange(0, -0.5, -0.05)])

    pose_list = []
    initial_pose = get_pose(box_attacher, euler=True)

    wpose_initial = box_attacher.move_group.get_current_pose().pose
    for x in xBound:
        position_change = np.array((x, 0, 0))
        translation = get_position(box_attacher, initial_pose, position_change)
        pose_list.append(translation)

    for y in yBound:
        position_change = np.array((0, y, 0))
        translation = get_position(box_attacher, initial_pose, position_change)
        pose_list.append(translation)

    for z in zBound:
        position_change = np.array((0, 0, z))
        translation = get_position(box_attacher, initial_pose, position_change)
        pose_list.append(translation)
    
    poses = get_calib_poses_new(box_attacher, initial_pose)
    pose_list = pose_list + poses

    return pose_list

    
