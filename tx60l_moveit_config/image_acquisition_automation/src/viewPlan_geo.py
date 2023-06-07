import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from src.data_robot_mover2 import *

json_path = "/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/poses.json"

def viewPlan(box_attacher):
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
            # current_orientation = self.move_group.get_current_pose().pose.orientation
            json_dict[pose]['position (x,y,z)'] = [str(current_pose.x), str(current_pose.y), str(current_pose.z)]
            json_dict[pose]['orintation (w,x,y,z)'] = [str(current_orientation.w), str(current_orientation.x), str(current_orientation.y), str(current_orientation.z)]
            json_dict[pose]['joint_goal'] = [str(a) for a in self.move_group.get_current_joint_values()]

            motion_successful = move_robot(self, pose_list[i])
            pose += 1
        
        # Serializing json
        json_object = json.dumps(json_dict, indent=4)
        # Writing to sample.json
        with open(json_path, "w") as outfile:
            outfile.write(json_object)


    pass

def viewPlan_list(box_attacher):
    xBound = np.arange(-0.5, 0.5, 0.1)
    yBound = np.arange(-0.5, 0.5, 0.1)
    zBound = np.arange(-0.5, 0.5, 0.1)

    pose_list = []
    initial_pose = get_pose(box_attacher, euler=True)

    wpose_initial = box_attacher.move_group.get_current_pose().pose
    for x in xBound:
        wpose = wpose_initial
        wpose.position.x = wpose.position.x + x
        position = np.array((wpose.position.x, wpose.position.y, wpose.position.z))
        translation = get_position(box_attacher, initial_pose, position)
        pose_list.append(translation)

    for y in yBound:
        wpose = wpose_initial
        wpose.position.y = wpose.position.y + y
        position = np.array((wpose.position.x, wpose.position.y, wpose.position.z))
        translation = get_position(box_attacher, initial_pose, position)
        pose_list.append(translation)

    for z in zBound:
        wpose = wpose_initial
        wpose.position.z = wpose.position.z + z
        position = np.array((wpose.position.x, wpose.position.y, wpose.position.z))
        translation = get_position(box_attacher, initial_pose, position)
        pose_list.append(translation)
    
    poses = get_calib_poses_new(box_attacher, initial_pose)
    pose_list = pose_list + poses

    
