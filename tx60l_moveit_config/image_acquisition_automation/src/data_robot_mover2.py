# https://gitlab.lrz.de/autotron-group/oguz_raptor/-/blob/master/pose_estimation/calibration_and_data_acquisition/data_robot_mover.py

#!/usr/bin/env python
# https://gitlab.lrz.de/autotron-group/oguz_raptor/-/blob/master/pose_estimation/calibration_and_data_acquisition/data_robot_mover.py
"""
This script runs in Python2. It implements the robot movement profiles for camera calibration,
3D model (reference map) creation and test poses for testing the pose estimation.
At each pose, the data_image_saver.py is being called which will acquire and save a set of images.

Please be aware of ROS 1 running on Python 2 and CVB on Python 3, therefore the workaround with subprocess.call() is
needed.
"""
import sys
import os
from cv2 import applyColorMap
import numpy as np
from math import pi
from math import radians
import time
from scipy.spatial.transform import Rotation as ScyRot 

filedir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(filedir, "..", ".."))

# import helpers
from .helpers import *
# from helper import vistools 
# from helper import hand_eye_calib 
# from helper import data_manager
# from helper import robot_tools  

# ros imports 
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

def get_pose(box_attacher, euler=False, print_msg=False, ros_format=False, extrinsic_euler=True, T_mm=False):
    """
    Returns and prints the current robot flange pose relative to robot base. 
    Position in meters; orientation in XYZ-Euler or as quaternion; 
    T_mm=True returns a 4x4 transformation matrix with translatory part in mm. 
    Errors: 
    https://groups.google.com/g/moveit-users/c/kZaVTrlsUc0 
    https://answers.ros.org/question/300978/movegroupcommander-get_current_pose-returns-incorrect-result-when-using-real-robot/ 
    """
    
    # get pose from moveit 
    wpose = box_attacher.move_group.get_current_pose().pose
    
    # get position 
    position = np.array([wpose.position.x, wpose.position.y, wpose.position.z], dtype="float64")
    # get orientation quaternion 
    orientation_quat = np.array([wpose.orientation.x, wpose.orientation.y, 
                                wpose.orientation.z, wpose.orientation.w], dtype="float64") 
    
    pose_dict = {}
    if(print_msg): 
        print("Robot flange position in m: ", position)
    if(ros_format): 
        # ros: xyz in m and orientation as quaternion 
        if(print_msg): 
            print("Robot flange orientation as quaternion: ", orientation_quat)
        return wpose 
    elif(T_mm): 
        T_b_f = unit_transform() 
        T_b_f[:3,:3] = ScyRot.from_quat(orientation_quat).as_matrix()
        T_b_f[:3,3] = position * 1000.0 
        return T_b_f
    elif(euler): 
        if(extrinsic_euler): 
            orientation_euler = ScyRot.from_quat([wpose.orientation.x, wpose.orientation.y, 
                            wpose.orientation.z, wpose.orientation.w]).as_euler('xyz', degrees=True)
            pose = np.concatenate([position, orientation_euler]) 
            pose_dict['position'] = position
            pose_dict['orientation'] = orientation_euler
            if(print_msg==True): 
                print("Robot flange orientation in xyz-Euler: ", orientation_euler) 
        else: 
        # euler 
            orientation_euler = ScyRot.from_quat([wpose.orientation.x, wpose.orientation.y, 
                            wpose.orientation.z, wpose.orientation.w]).as_euler('XYZ', degrees=True)
            pose = np.concatenate([position, orientation_euler]) 
            pose_dict['position'] = position
            pose_dict['orientation'] = orientation_euler
            if(print_msg==True): 
                print("Robot flange orientation in XYZ-Euler: ", orientation_euler) 
    else: 
        # quaternion
        pose = np.concatenate([position, orientation_quat])
        pose_dict['position'] = position
        pose_dict['orientation'] = orientation_quat
        if(print_msg): 
            print("Robot flange orientation as quaternion: ", orientation_quat) 
        
    return pose_dict 

def get_calib_poses(box_attacher, initial_pose, offsetpoint=[0.0, 0.0, 0.0], angles=None): 
    """
    Creates a list of robot endeffector motion poses for calibration
    
    Input 
    - initial_pose [dict]:  initial flange pose relative to base; 
                            is used as the center point for all calibration motions 
                            initial_pose["position"] = [x, y, z] in meters; 
                            initial_pose["orientation"] = [rx, ry, rz] in xyz Euler;
    - offsetpoint (3x1-list; float):    rotate around this offset point; in m; default: no offset 
    - angles (3xn float list):  rotation angle lists for xyz axis in Euler. 
                                Default is +-10° around every axis. 
                                [[x angles], [y angles], [z angles]] 

    """

    # create calibration poses 
    pose_list = []  
    # use default rotation angles if no angle lists are given
    if(angles==None): 
        # euler rotations around each axis relative to the initial pose; 0.0 is automatically included 
        # angle_x_list = [0, 10.0, 15.0, 20, -10.0, -15.0, -20] 
        # angle_y_list = [0, 10.0, 15.0, 20, -10.0, -15.0, -20]#, -10.0] 
        # angle_z_list = [0, 10.0, 15.0, 20, -10.0, -15.0, -20] 
        angle_x_list = [0.0, 10.0, -10.0] 
        angle_y_list = [0.0, 10.0, 15.0]#, -10.0] 
        angle_z_list = [0.0, 10.0, -10.0] 
    else:
        angle_x_list = angles[0] 
        angle_y_list = angles[1] 
        angle_z_list = angles[2] 

    # list of resulting poses for the robot in meters 
    T_b_f_rot_list = [] 
    # offset point 
    T_f_m = euler2T( np.array(offsetpoint, dtype="float"),  
                                        ScyRot.from_euler('xyz', [0,0,0], degrees=True).as_matrix() )
    # initial pose as matrix 
    T_init = euler2T(initial_pose["position"], initial_pose["orientation"], extrinsic=True) 

    # create poses
    # around x-axis 
    for rx in angle_x_list:  
        # around y-axis 
        for ry in angle_y_list:  
            # around z-axis 
            for rz in angle_z_list: 
                # current xyz-euler angles relative initial pose  
                angles_tmp = [rx, ry, rz] 
                # euler angles (rx, ry, rz) to quaternions (x, y, z, w); extrinsic rotation; 
                # transform for rotion around point 
                # T_rot = euler2T( np.array([0.0,0.0,0.0], dtype="float"), 
                #                                 ScyRot.from_euler('XYZ', [angles_tmp[0], angles_tmp[1], angles_tmp[2]], 
                #                                                             degrees=True).as_matrix() ) 
                T_rot = euler2T( np.array([0.0,0.0,0.0], dtype="float"), 
                                                ScyRot.from_euler('xyz', [angles_tmp[0], angles_tmp[1], angles_tmp[2]], 
                                                                            degrees=True).as_matrix() ) 
                # Rotate around robot flange: T_b_f * T_f_m * T_rot * T_m_f
                T_b_f_rot = np.linalg.multi_dot( [T_init, T_f_m, T_rot, np.linalg.inv(T_f_m)] ) 
                T_b_f_rot_list.append(T_b_f_rot) 
    
    return T_b_f_rot_list


def get_orientation(box_attacher, initial_pose, rotation_degree):
    '''
    Input: initial_pose, rotation_degree[x,y,z]
    Output: transformation matrix
    '''
    T_init = euler2T(initial_pose["position"], initial_pose["orientation"], extrinsic=True)
    T_rot = euler2T(np.array([0.0, 0.0, 0.0], dtype="float"),
                    ScyRot.from_euler('xyz', [rotation_degree[0], rotation_degree[1], rotation_degree[2]],
                                      degrees=True).as_matrix())
    T_b_f_rot = np.linalg.multi_dot([T_init, T_rot])

    return T_b_f_rot

def get_position(box_attacher, initial_pose, position, offsetpoint=[0.0, 0.0, 0.0]):
    '''
    position -> numpy array
    '''
    T_init = euler2T(initial_pose["position"], initial_pose["orientation"], extrinsic=True)
    T_translation = euler2T(position, np.array([0.0, 0.0, 0.0], dtype="float"), extrinsic=True)
    # offset point
    T_f_m = euler2T(np.array(offsetpoint, dtype="float"),
                    ScyRot.from_euler('xyz', [0, 0, 0], degrees=True).as_matrix())
    
    T_b_f_translation = np.linalg.multi_dot([T_init, T_f_m, T_translation, np.linalg.inv(T_f_m)])

    return T_b_f_translation


def get_calib_poses_new(box_attacher, initial_pose, offsetpoint=[0.0, 0.0, 0.0], angles=None):
    """
    Creates a list of robot endeffector motion poses for calibration

    Input
    - initial_pose [dict]:  initial flange pose relative to base;
                            is used as the center point for all calibration motions
                            initial_pose["position"] = [x, y, z] in meters;
                            initial_pose["orientation"] = [rx, ry, rz] in xyz Euler;
    - offsetpoint (3x1-list; float):    rotate around this offset point; in m; default: no offset
    - angles (3xn float list):  rotation angle lists for xyz axis in Euler.
                                Default is +-10° around every axis.
                                [[x angles], [y angles], [z angles]]

    """

    # create calibration poses
    pose_list = []
    # use default rotation angles if no angle lists are given
    if (angles == None):
        # euler rotations around each axis relative to the initial pose; 0.0 is automatically included
        # angle_x_list = [0, 10.0, 20, 30, 40, -10.0, -20, -30, -40]
        # angle_y_list = [0, 10.0, 20, 30, 40, -10.0, -20, -30, -40]  # , -10.0]
        # angle_z_list = [0, 10.0, 20, 30, 40, -10.0, -20, -30, -40]
        angle_x_list = [0]
        angle_y_list = [0]  # , -10.0]
        z1 = np.arange(0, 290.0, 5)
        z2 = np.arange(0, -290.0, -5)
        angle_z_list = np.concatenate((z1, z2))
        # angle_z_list = [0, 20.0, 40.0, 60.0, 80.0, 100.0, 110.0, 120.0, 130.0, 140.0, 150.0, 160.0, 
        #                 170.0, 180.0, 190.0, 200.0, 210.0, 220.0, 230.0, 250.0, 260.0, 280.0, -20.0, 
        #                 -40.0, -60.0, -80.0, -100.0, -110.0, -120.0, -130.0, -140.0, -150.0, -160.0, 
        #                 -170.0, -180.0, -190.0, -200.0, -210.0, -220.0, -230.0, -250.0, -260.0, -280.0]
        # angle_x_list = [0.0, 10.0]
        # angle_y_list = [0.0, 10.0]#, -10.0]
        # angle_z_list = [0.0, 10.0]
    else:
        angle_x_list = angles[0]
        angle_y_list = angles[1]
        angle_z_list = angles[2]

        # list of resulting poses for the robot in meters
    T_b_f_rot_list = []
    # offset point
    T_f_m = euler2T(np.array(offsetpoint, dtype="float"),
                    ScyRot.from_euler('xyz', [0, 0, 0], degrees=True).as_matrix())
    # initial pose as matrix
    T_init = euler2T(initial_pose["position"], initial_pose["orientation"], extrinsic=True)

    # create poses
    # around z-axis
    for rz in angle_z_list:
        # current xyz-euler angles relative initial pose
        angles_tmp = [0, 0, rz]
        # euler angles (rx, ry, rz) to quaternions (x, y, z, w); extrinsic rotation;
        # transform for rotion around point
        # T_rot = euler2T( np.array([0.0,0.0,0.0], dtype="float"),
        #                                 ScyRot.from_euler('XYZ', [angles_tmp[0], angles_tmp[1], angles_tmp[2]],
        #                                                             degrees=True).as_matrix() )
        T_rot = euler2T(np.array([0.0, 0.0, 0.0], dtype="float"),
                        ScyRot.from_euler('xyz', [angles_tmp[0], angles_tmp[1], angles_tmp[2]],
                                          degrees=True).as_matrix())
        # Rotate around robot flange: T_b_f * T_f_m * T_rot * T_m_f
        T_b_f_rot = np.linalg.multi_dot([T_init, T_f_m, T_rot, np.linalg.inv(T_f_m)])
        T_b_f_rot_list.append(T_b_f_rot)

    T_b_f_rot_list.append(T_init)

    # around y-axis
    for ry in angle_y_list:
        angles_tmp = [0, ry, 0]
        # euler angles (rx, ry, rz) to quaternions (x, y, z, w); extrinsic rotation;
        # transform for rotion around point
        # T_rot = euler2T( np.array([0.0,0.0,0.0], dtype="float"),
        #                                 ScyRot.from_euler('XYZ', [angles_tmp[0], angles_tmp[1], angles_tmp[2]],
        #                                                             degrees=True).as_matrix() )
        T_rot = euler2T(np.array([0.0, 0.0, 0.0], dtype="float"),
                        ScyRot.from_euler('xyz', [angles_tmp[0], angles_tmp[1], angles_tmp[2]],
                                          degrees=True).as_matrix())
        # Rotate around robot flange: T_b_f * T_f_m * T_rot * T_m_f
        T_b_f_rot = np.linalg.multi_dot([T_init, T_f_m, T_rot, np.linalg.inv(T_f_m)])
        T_b_f_rot_list.append(T_b_f_rot)

    T_b_f_rot_list.append(T_init)
    # around x-axis
    for rx in angle_x_list:
        angles_tmp = [rx, 0, 0]
        # euler angles (rx, ry, rz) to quaternions (x, y, z, w); extrinsic rotation;
        # transform for rotion around point
        # T_rot = euler2T( np.array([0.0,0.0,0.0], dtype="float"),
        #                                 ScyRot.from_euler('XYZ', [angles_tmp[0], angles_tmp[1], angles_tmp[2]],
        #                                                             degrees=True).as_matrix() )
        T_rot = euler2T(np.array([0.0, 0.0, 0.0], dtype="float"),
                        ScyRot.from_euler('xyz', [angles_tmp[0], angles_tmp[1], angles_tmp[2]],
                                          degrees=True).as_matrix())
        # Rotate around robot flange: T_b_f * T_f_m * T_rot * T_m_f
        T_b_f_rot = np.linalg.multi_dot([T_init, T_f_m, T_rot, np.linalg.inv(T_f_m)])
        T_b_f_rot_list.append(T_b_f_rot)

    T_b_f_rot_list.append(T_init)

    return T_b_f_rot_list

def move_robot(box_attacher, pose, extrinsic_euler=True): #TODO save_motionplan=None, load_motionplan=None
    """"
    TODO relative movements (get current position and move relative to it). 

    pose (dict or 4x4-np):  pose["position"] = [x, y, z], pose["orientation"] = [rx, ry, rz, rw]; 
                            if pose["orientation"] has only 3 elemens, then "XYZ" euler angles are assumed;  
                            "XYZ"-euler-rotation begins rotations from the back, i.e. with "Z" etc. 
                            Position in meters. 
    move_group (MoveObject): 
    """

    wpose = geometry_msgs.msg.Pose()
    wpose1 = box_attacher.move_group.get_current_pose().pose

    # check input pose type 
    if(type(pose)==type(np.array([]))): 
        if (pose.shape == (4,4)): 
            # homogenous transformation matrix 
            # position 
            wpose.position.x = pose[0,3] 
            wpose.position.y = pose[1,3] 
            wpose.position.z = pose[2,3]  
            # orientation 
            _orientation = ScyRot.from_matrix(pose[:3,:3]).as_quat()
            wpose.orientation.x = _orientation[0] 
            wpose.orientation.y = _orientation[1] 
            wpose.orientation.z = _orientation[2] 
            wpose.orientation.w = _orientation[3] 
    elif (len(pose["orientation"]) == 4): 
        # position 
        wpose.position.x = pose["position"][0] 
        wpose.position.y = pose["position"][1] 
        wpose.position.z = pose["position"][2]  
        # quaternions
        wpose.orientation.x = pose["orientation"][0] 
        wpose.orientation.y = pose["orientation"][1] 
        wpose.orientation.z = pose["orientation"][2] 
        wpose.orientation.w = pose["orientation"][3] 
    elif(len(pose["orientation"]) == 3):
        # position 
        wpose.position.x = pose["position"][0] 
        wpose.position.y = pose["position"][1] 
        wpose.position.z = pose["position"][2]  
        # euler "XYZ"  
        # euler to quaternion 
        if(extrinsic_euler): 
            orientation_quat = ScyRot.from_euler('xyz', pose["orientation"], degrees=True).as_quat()
        else: 
            orientation_quat = ScyRot.from_euler('XYZ', pose["orientation"], degrees=True).as_quat() 
        wpose.orientation.x = orientation_quat[0] 
        wpose.orientation.y = orientation_quat[1] 
        wpose.orientation.z = orientation_quat[2] 
        wpose.orientation.w = orientation_quat[3] 

    box_attacher.move_group.clear_pose_targets() 
    
    # moveit options 
    plan = False
    if(True): 
        box_attacher.move_group.set_pose_target(wpose)
        plan = box_attacher.move_group.go(wait=True)
    else: 
        Plan = box_attacher.move_group.plan(wpose) 
        box_attacher.move_group.execute(Plan[1]) 
    box_attacher.move_group.stop() 

    if not plan:
        box_attacher.move_group.clear_pose_targets() 
        box_attacher.move_group.stop() 
        print("[ERROR] No motion plan found.")

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    # Problem: move_group.go(wait=True) does not halt the programm until
    #          the move has been executed
    #          If you clear_pose_targets() you might loose some moves
    box_attacher.move_group.clear_pose_targets() 
    # self.current_pose = box_attacher.get_pose(euler=False) 

    return plan 