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

import helper
from helper import mathfunctions 
from helper import vistools 
from helper import hand_eye_calib 
from helper import data_manager
from helper import robot_tools  

# ros imports 
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf


class MoveObject(object):
    """
    Creates an object connecting to the Staeubli robot and therefore representing it. 
    kinematic solver: https://ros-planning.github.io/moveit_tutorials/doc/kinematics_configuration/kinematics_configuration_tutorial.html 
    jerk motion: https://github.com/ros-industrial/staubli_val3_driver/issues/2#issuecomment-563364169 
    """
    def __init__(self, datapath=""): 
        super(MoveObject, self).__init__()
        # cleaned_args are needed because ptvsd-debugger messes with argv and confuses rospy_initialize
        # issue should be resolved in later vscode versions
        cleaned_args = [a for a in sys.argv if not os.path.basename(__file__) in os.path.basename(__file__)]
        moveit_commander.roscpp_initialize(cleaned_args)
        rospy.init_node('main_node', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        # set robot motion planner parameters 
        self.move_group.set_planner_id("TRRT") # good: "TRRT", "PDST"
        self.move_group.set_goal_orientation_tolerance(0.0001) 
        self.move_group.set_goal_position_tolerance(0.000001) # in m; default 1 µm 
        self.move_group.set_goal_joint_tolerance(0.00001)
        self.move_group.set_planning_time(10.0) # in seconds
        
        # initial robot pose for camera calibration 
        self.initial_calibration_pose = {"position":[0.102, -0.427, 0.805], "orientation":[0, 98, -45]}
        print("Initial calibration pose reached.")

        self.group_names = self.robot.get_group_names()
        self.datapath = datapath 
        print("Initial pose:") 
        self.current_pose = self.get_pose(euler=False, print_msg=False) 

        # inits
        self.tracker = None 
        self.gripper = None 
        self.config = None 
        self.rootpath = None 
        self.datapath = None  
        
        # init hand-eye calib data structures; 
        # only cam-specific transforms are a list for every cam pair;  
        self.T_c_b = []  
        self.T_b_c = []
        self.T_f_s = None 
        self.T_s_f = None
        self.T_l_c = [] 
        self.T_r_s = None


    def calibrate_handeye(self, motion_pattern_list, datapath, 
                            hand_eye_calibpath="", time_string_dir="000", onlycam_list=[False]): 
        """ 
        Hand-eye calibration wrapper. 

        Args: 
        - motion_pattern (2d list):     List of list of robot motion poses (T_b_f). 
                                        [[T_b_f_list for cam0], [T_b_f_list for cam1], ...]; 
        - datapath (string):            path of the general data directory 
        - hand_eye_calibpath (string):  Path for loading calibration data. If empty, new data is collected.
        - time_string (string):         Timestamp for current measurement round. Used for path names.  
        - onlycam_list (bool list):     If True, then only cam data is used; everything otherwise; 
                                        One list element per camera pair; 
        
        Result: 
        - T_f_s (4x4, np):  transformation matrix from flange to sample
        - T_c_b (4x4, np):  transformation matrix from camera to base 
        - T_l_c (4x4, np):  transformation matrix from laser tracker to camera; None if onlycam=True
        - T_r_s (4x4, np):  transformation matrix from relfector to sample; None if onlycam=True

        """
        for motion_pattern, onlycam, campair_idx in zip(motion_pattern_list, onlycam_list, range(len(onlycam_list))): 
            T_f_s, T_c_b, T_l_c, T_r_s = hand_eye_calib.calibrate(self, motion_pattern, datapath, campairs=[campair_idx], 
                                                            hand_eye_calibpath=hand_eye_calibpath, 
                                                            time_string_dir=time_string_dir, onlycam=onlycam)
            # save transforms into robot object 
            self.T_c_b.append(T_c_b) 
            self.T_b_c.append(np.linalg.inv(T_c_b)) 
            self.T_f_s = T_f_s 
            self.T_s_f = np.linalg.inv(T_f_s) 
            self.T_l_c.append(T_l_c) 
            self.T_r_s = T_r_s 
            if(onlycam==False):
                self.T_c_l.append(np.linalg.inv(T_l_c)) 
                self.T_s_r = np.linalg.inv(T_r_s) 


    def get_pose(self, euler=False, print_msg=False, ros_format=False, extrinsic_euler=True, T_mm=False):
        """
        Returns and prints the current robot flange pose relative to robot base. 
        Position in meters; orientation in XYZ-Euler or as quaternion; 
        T_mm=True returns a 4x4 transformation matrix with translatory part in mm. 
        Errors: 
        https://groups.google.com/g/moveit-users/c/kZaVTrlsUc0 
        https://answers.ros.org/question/300978/movegroupcommander-get_current_pose-returns-incorrect-result-when-using-real-robot/ 
        """
        
        # get pose from moveit 
        wpose = self.move_group.get_current_pose().pose
        
        # get position 
        position = np.array([wpose.position.x, wpose.position.y, wpose.position.z], dtype="float64")
        # get orientation quaternion 
        orientation_quat = np.array([wpose.orientation.x, wpose.orientation.y, 
                                    wpose.orientation.z, wpose.orientation.w], dtype="float64") 
        
        if(print_msg): 
            print("Robot flange position in m: ", position)
        if(ros_format): 
            # ros: xyz in m and orientation as quaternion 
            if(print_msg): 
                print("Robot flange orientation as quaternion: ", orientation_quat)
            return wpose 
        elif(T_mm): 
            T_b_f = mathfunctions.unit_transform() 
            T_b_f[:3,:3] = ScyRot.from_quat(orientation_quat).as_matrix()
            T_b_f[:3,3] = position * 1000.0 
            return T_b_f
        elif(euler): 
            if(extrinsic_euler): 
                orientation_euler = ScyRot.from_quat([wpose.orientation.x, wpose.orientation.y, 
                                wpose.orientation.z, wpose.orientation.w]).as_euler('xyz', degrees=True)
                pose = np.concatenate([position, orientation_euler]) 
                if(print_msg==True): 
                    print("Robot flange orientation in xyz-Euler: ", orientation_euler) 
            else: 
            # euler 
                orientation_euler = ScyRot.from_quat([wpose.orientation.x, wpose.orientation.y, 
                                wpose.orientation.z, wpose.orientation.w]).as_euler('XYZ', degrees=True)
                pose = np.concatenate([position, orientation_euler]) 
                if(print_msg==True): 
                    print("Robot flange orientation in XYZ-Euler: ", orientation_euler) 
        else: 
            # quaternion
            pose = np.concatenate([position, orientation_quat])
            if(print_msg): 
                print("Robot flange orientation as quaternion: ", orientation_quat) 
            
        return pose  

    
    def move_robot_joints(self, angles, relative=False): 
        """
        Relative movements possible (get current position and move relative to it). 
        Moves the joints of the robot. 
        
        Args: 
        - angles (np, 6):       joint Euler angles for joint 1..6; 
                                if you don't want to change a joint then set its value to None;  
        - relative (bool):      if True: moves joints relative to their current value; 
                                absolute values are used otherwise; 
                                
        Returns 
        - plan (bool):          True if motion was successful 
        
        """
        # get the current joint values from the robot 
        joint_goal = self.move_group.get_current_joint_values()
        if relative==True: #TODO fix euler to rad problem 
            # move joints relative to current joint position
            # check if input is within joint limits 
            if -180.0<=np.rad2deg(joint_goal[0])+angles[0]<=180.0 and -127.5<=np.rad2deg(joint_goal[1])+angles[1]<=127.5 and \
                -152.5<=np.rad2deg(joint_goal[2])+angles[2]<=152.5 and -270.0<=np.rad2deg(joint_goal[3])+angles[3]<=270.0 and \
                -121.0<=np.rad2deg(joint_goal[4])+angles[4]<=121.0 and -270.0<=np.rad2deg(joint_goal[5])+angles[5]<=270.0:  
                # leave the old value if the new value is None 
                joint_goal[0] = joint_goal[0]+np.deg2rad(angles[0]) if angles[0]!=None else joint_goal[0]
                joint_goal[1] = joint_goal[1]+np.deg2rad(angles[1]) if angles[1]!=None else joint_goal[1]
                joint_goal[2] = joint_goal[2]+np.deg2rad(angles[2]) if angles[2]!=None else joint_goal[2]
                joint_goal[3] = joint_goal[3]+np.deg2rad(angles[3]) if angles[3]!=None else joint_goal[3]
                joint_goal[4] = joint_goal[4]+np.deg2rad(angles[4]) if angles[4]!=None else joint_goal[4]
                joint_goal[5] = joint_goal[5]+np.deg2rad(angles[5]) if angles[5]!=None else joint_goal[5]
            else: 
                raise ValueError('Joint angle limits exceeded. Limits: [+-180, +-127.5, +-152.5, +-270, +-121, +-270]')
        else: 
            # move joints in absolute angle values 
            # check if input is within joint limits 
            if -180.0<=angles[0]<=180.0 and -127.5<=angles[1]<=127.5 and -152.5<=angles[2]<=152.5 and \
                -270.0<=angles[3]<=270.0 and -121.0<=angles[4]<=121.0 and -270.0<=angles[5]<=270.0:  
                # leave the old value if the new value is None 
                joint_goal[0] = np.deg2rad(angles[0]) if angles[0]!=None else joint_goal[0]
                joint_goal[1] = np.deg2rad(angles[1]) if angles[1]!=None else joint_goal[1]
                joint_goal[2] = np.deg2rad(angles[2]) if angles[2]!=None else joint_goal[2]
                joint_goal[3] = np.deg2rad(angles[3]) if angles[3]!=None else joint_goal[3]
                joint_goal[4] = np.deg2rad(angles[4]) if angles[4]!=None else joint_goal[4]
                joint_goal[5] = np.deg2rad(angles[5]) if angles[5]!=None else joint_goal[5]
            else: 
                raise ValueError('Only the following values are allowed: [+-180, +-127.5, +-152.5, +-270, +-121, +-270]')
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        plan = self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
        
        return plan 
    
    
    
    def move_robot(self, pose, extrinsic_euler=True): #TODO save_motionplan=None, load_motionplan=None
        """"
        TODO relative movements (get current position and move relative to it). 

        pose (dict or 4x4-np):  pose["position"] = [x, y, z], pose["orientation"] = [rx, ry, rz, rw]; 
                                if pose["orientation"] has only 3 elemens, then "XYZ" euler angles are assumed;  
                                "XYZ"-euler-rotation begins rotations from the back, i.e. with "Z" etc. 
                                Position in meters. 
        move_group (MoveObject): 
        """

        wpose = geometry_msgs.msg.Pose()

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

        self.move_group.clear_pose_targets() 
        
        # moveit options 
        plan = False
        if(True): 
            self.move_group.set_pose_target(wpose)
            plan = self.move_group.go(wait=True)
        else: 
            Plan = self.move_group.plan(wpose) 
            self.move_group.execute(Plan[1]) 
        self.move_group.stop() 

        if not plan:
            self.move_group.clear_pose_targets() 
            self.move_group.stop() 
            print("[ERROR] No motion plan found.")

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        # Problem: move_group.go(wait=True) does not halt the programm until
        #          the move has been executed
        #          If you clear_pose_targets() you might loose some moves
        self.move_group.clear_pose_targets() 
        self.current_pose = self.get_pose(euler=False) 

        return plan 
    

    def correct_motion(self, evalposenum, T_c_s_target, 
                       t_euclid_threshold=0.050, euler_threshold=0.5, 
                       visualize=True, synthetic=None, campairs=[0]): 
        """
        Motion compensation wrapper. Several attempts to test lower thresholds and to retry after failure. 
        
        Args: 
        - evalposenum (int):            current target pose number 
        - T_f_s (np, 4x4):              Transformation between flange and sample. Also called T_e_s. 
        - t_euclid_threshold (float):   Motion correction stops when this tranlatory error threshold has been reached. 
        - euler_threshold (float):      Motion correction stops when this rotatory error threshold has been reached. 
        - synthetic (dict):             If not None then synthetic data is used. 
                                        Dict should contain R_c_s, t_c_s. 
        
        Returns: 
        - T_c_s (np-4x4):
        - t_euclid_error (float):       Translatory motion error after motion correction; 
        - motion_success (bool):
        - compensation_list (list):     evalposenum; compstep; t_euclid_error; t_c_s_x; t_c_s_y; t_c_s_z; 
                                        t_c_s_target_x; t_c_s_target_y; t_c_s_target_z
        
        """
        attempts = 3 # number of compensation attempts 
        # several attempts to compensate if necessary 
        for a in range(attempts): 
            print("\n[CORRECTION][Target "+str(evalposenum)+"][Attempt "+str(a)+"] Starting.\n")
            T_c_s, t_euclid_error, motion_success, compensation_list, comp_success = self._correct_motion(evalposenum, 
                                                                                a, 
                                                                                T_c_s_target, 
                                                                                t_euclid_threshold=t_euclid_threshold, 
                                                                                euler_threshold=euler_threshold, 
                                                                                visualize=visualize, 
                                                                                synthetic=synthetic, 
                                                                                max_steps=5, 
                                                                                campairs=campairs)
            # stop compensation attempts if last compensation was successful 
            if(comp_success):
                print("\n[CORRECTION][Target "+str(evalposenum)+"][Attempt "+str(a)+"] Successful.\n")
                break 
        
        # in case if all attempts were not successful
        if(comp_success==False):
            print("\n[CORRECTION][Target "+str(evalposenum)+"][ERROR] Compensation failed.\n")

        return T_c_s, t_euclid_error, motion_success, compensation_list 

    
    def _correct_motion(self, evalposenum, a, T_c_s_target, 
                       t_euclid_threshold=0.050, euler_threshold=0.5, 
                       visualize=True, synthetic=None, max_steps=5, campairs=[0]): 
        """
        #TODO compensate according to measurement cosy (T_c_m) and not sample cosy (T_c_s) 
        Closed-loop motion correction. Computes difference between actual and target pose. 
        
        Args: 
        - evalposenum (int):            current target pose number 
        - a (int):                      Attempt for current target pose s
        - T_f_s (np, 4x4):              Transformation between flange and sample. Also called T_e_s. 
        - t_euclid_threshold (float):   Motion correction stops when this tranlatory error threshold has been reached. 
        - euler_threshold (float):      Motion correction stops when this rotatory error threshold has been reached. 
        - synthetic (dict):             If not None then synthetic data is used. 
                                        Dict should contain R_c_s, t_c_s. 
        - max_steps (int):           Maximum number of compensation steps 

        Returns: 
        - T_c_s (np-4x4):
        - t_euclid_error (float):       Translatory motion error after motion correction; 
        - motion_success (bool):
        - compensation_list (list):     evalposenum; compstep; t_euclid_error; t_c_s_x; t_c_s_y; t_c_s_z; 
                                        t_c_s_target_x; t_c_s_target_y; t_c_s_target_z
        - comp_success (bool):          Was compensation successful? 
        
        """
        # correction loop until difference between actual and target sample pose is small enough 
        correction_steps = -1
        motion_success = False 
        t_c_s_list = [] 
        compensation_list = [] # compstep, t_euclid_error, t_c_s, t_c_s_target  
        comp_success = False 
        T_c_s = None 
        t_euclid_error = 999999.9 
        euler_error = 999999.9 
        while(True): # until target pose has been reached
            correction_steps = correction_steps + 1
            # stop motion correction if too many correction steps have been made 
            if(correction_steps==max_steps): 
                print("\n[CORRECTION][Target "+str(evalposenum)+"][Attempt "+str(a)+"][Correction "+str(correction_steps)+"] Stopping because of too many correction steps.\n")
                break 
            # estimate current pose of the sample 
            print("\n[CORRECTION][Target "+str(evalposenum)+"][Attempt "+str(a)+"][Correction "+str(correction_steps)+"] Pose estimation begins.")
            if(synthetic!=None): 
                # use synthetic data for pose estimation
                R_c_s = synthetic["R_c_s"] 
                t_c_s = synthetic["t_c_s"]  
            else: 
                # use real data for pose estimation 
                try: 
                    R_c_s, t_c_s = self.tracker.get_sample_pose(impairnum=correction_steps, campairs=campairs)
                except Exception as error: 
                    print("\n[CORRECTION][Target "+str(evalposenum)+"][Attempt "+str(a)+"][Correction "+str(correction_steps)+"] Not enough 3d-3d-matches found.\n")
                    continue
            T_c_s = mathfunctions.euler2T(t_c_s, R_c_s)
            t_c_s_list.append(T_c_s[:3,3]) 

            ### difference between measurement points during rotation ##
            # get reference T_c_m 
            if(evalposenum==0): 
                self.T_c_m_ref = np.linalg.multi_dot([ T_c_s, self.T_s_m ]) 
            else: 
                # get current measurement point pose 
                T_c_m_tmp = np.linalg.multi_dot([ T_c_s, self.T_s_m ]) 
                # compare T_c_m_tmp to first value T_c_m_ref 
                T_c_m_diff = np.linalg.multi_dot([ np.linalg.inv(self.T_c_m_ref), T_c_m_tmp ]) 
                t_c_m_diff_norm = np.linalg.norm(T_c_m_diff[:3,3]) 
                # print("\n[CORRECTION][Target "+str(evalposenum)+"][Attempt "+str(a)+"][Correction "+str(correction_steps)+"] t_c_m_diff_norm="+str(t_c_m_diff_norm)+"\n") 

            # show compensation steps 
            if(visualize): 
                # leave out first measurement because the distance is too big 
                if(correction_steps>0):
                    try: 
                        vistools.show_compensation_points(correction_steps, targetpoint=np.array([T_c_s_target[:3,3]], dtype="float"), 
                                                            comppoints=np.array(t_c_s_list[1:], dtype="float")) 
                    except Exception as error: 
                        print("\n[CORRECTION][Target "+str(evalposenum)+"][Attempt "+str(a)+"][Correction "+str(correction_steps)+"] No visualization possible.\n")

            # motion error: difference between actual and target sample pose relative to cameras 
            T_s_c = np.linalg.inv(T_c_s)
            T_s_s_target = np.matmul(T_s_c, T_c_s_target) 
            # position error in mm 
            t_euclid_error = np.linalg.norm(T_s_s_target[:3,3])
            # orientation error in xyz-extrinsic euler angles 
            euler_error = ScyRot.from_matrix(T_s_s_target[:3,:3]).as_euler("xyz", degrees=True) # np.array([0, 0, 0])
            compensation_list.append( [evalposenum]+[correction_steps]+[t_euclid_error]+T_c_s[:3,3].tolist()+T_c_s_target[:3,3].tolist() )
            print("[CORRECTION][Target "+str(evalposenum)+"][Attempt "+str(a)+"][Correction "+str(correction_steps)+"] Position error: "+str(t_euclid_error)+" mm")
            print("[CORRECTION][Target "+str(evalposenum)+"][Attempt "+str(a)+"][Correction "+str(correction_steps)+"] Orientation error (rx°, ry°, rz°): ", str(euler_error),"\n") 

            # check if the sample is close enough to the target
            if (t_euclid_error < t_euclid_threshold and (euler_error < euler_threshold).all()):
                comp_success = True 
                print("\n[CORRECTION][Target "+str(evalposenum)+"][Attempt "+str(a)+"][Correction "+str(correction_steps)+"] Target pose has been reached.")
                # leave correction loop 
                break 
            else: 
                # correct motion error
                print("[CORRECTION][Target "+str(evalposenum)+"][Attempt "+str(a)+"][Correction "+str(correction_steps)+"] Motion correction begins.")
                motion_success = self.motion_correction_algorithm(correction_steps, T_c_s, T_c_s_target, 
                                                                    T_s_s_target, synthetic=synthetic, campair_idx=campairs[0]) 
                # stop attempt if no motion plan could be found 
                if(motion_success==False): 
                    print("\n[CORRECTION][Target "+str(evalposenum)+"][Attempt "+str(a)+"][Correction "+str(correction_steps)+"] No motion plan could be found. Skipping attempt.\n")
                    break 

        print("[CORRECTION][Target "+str(evalposenum)+"][Attempt "+str(a)+"][Correction "+str(correction_steps)+"] Final position error: "+str(t_euclid_error)+" mm.")
        print("[CORRECTION][Target "+str(evalposenum)+"][Attempt "+str(a)+"][Correction "+str(correction_steps)+"] Final orientation error (rx°, ry°, rz°): ", str(euler_error),"\n") 
        print("") 

        return T_c_s, t_euclid_error, motion_success, compensation_list, comp_success


    def motion_correction_algorithm(self, correction_steps, T_c_s, T_c_s_target, T_s_s_target, synthetic=None, campair_idx=0): 
        """
        Contains the motion correction algorithm and moves the robot towards the target pose. 
        The cameras work with mm and the robot with m. Don't forget to transform units! 
        Endeffector is used as a term for the robot flange. "e" == "f". 
        T_b_f: robot base to flange 
        T_b_s: robot base to sample
        T_b_f_T_b_f_target: difference between actual and target flange pose
        T_b_f_target: robot base to endffector target pose; 
                        this is also the new motion command for the robot; 
        T_b_s_targets: robot base to sample target pose 
        T_s_s_target: difference between actual and target sample pose relative to cameras

        ############## PID controller ##############
        http://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/Manual-Control.html 
        https://github.com/krishauser/Klampt
        https://pypi.org/project/simple-pid/
        https://jckantor.github.io/CBE30338/04.01-Implementing_PID_Control_with_Python_Yield_Statement.html
        https://onion.io/2bt-pid-control-python/
        https://studywolf.wordpress.com/2015/06/27/operational-space-control-of-6dof-robot-arm-with-spiking-cameras-part-2-deriving-the-jacobian/ 


        Args
        - compstep (int):   Compensation step 
        - T_c_s:            camera to sample  
        - T_c_s_target:     camera to sample target pose 
        - T_f_s:            robot flange to sample 
        - synthetic (dict): If not None then synthetic data is used. 
                            Dict should contain T_b_f (np 4x4 in mm), motion_success (bool).  

        Returns 
        - motion_success (bool): True if motion plan was found and robot could move. False otherwise. 
        
        """
        #TODO first compensation step without camera data 
                
        # get current flange pose relative to robot base
        if(synthetic!=None): 
            # use synthetic data 
            T_b_f = synthetic["T_b_f"] 
        else: 
            # use real data
            wpose = self.get_pose()  
            T_b_f = helper.mathfunctions.euler2T(wpose[:3], wpose[3:]) 
            # m to mm; just the translation part of the homogeneous matrix! 
            T_b_f[:3,3] = T_b_f[:3,3] * 1000.0
        
        # add difference between T_c_s and T_c_s_target to T_c_s_target to get a modified target 
        if(correction_steps == 0): 
            T_c_s_target_corrected = T_c_s_target.copy() 
        else: 
            T_c_s_target_corrected = np.matmul(T_c_s_target, T_s_s_target)

        # from camera to flange target pose 
        # T_c_f_target = T_c_s_target_corrected * T_s_f
        T_c_f_target = np.matmul(T_c_s_target_corrected, self.T_s_f) 

        # from robot base to flange target pose 
        # T_b_f_target = T_b_c * T_c_f_target
        T_b_f_target = np.matmul(self.T_b_c[campair_idx], T_c_f_target)

        # mm to m; just the translation part of the homogeneous matrix! 
        T_b_f_target[:3,3] = T_b_f_target[:3,3] / 1000.0

        # move robot to target flange pose 
        if(synthetic!=None): 
            # synthetic data 
            motion_success = synthetic["motion_success"]   
        else: 
            # use real data 
            robot_target_pose = {"position":T_b_f_target[:3,3], 
                                "orientation":ScyRot.from_matrix(T_b_f_target[:3,:3]).as_quat()}
            motion_success = self.move_robot(robot_target_pose)

        return motion_success
    
    
    def get_calib_poses(self, initial_pose, offsetpoint=[0.0, 0.0, 0.0], angles=None): 
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
        T_f_m = mathfunctions.euler2T( np.array(offsetpoint, dtype="float"),  
                                            ScyRot.from_euler('xyz', [0,0,0], degrees=True).as_matrix() )
        # initial pose as matrix 
        T_init = mathfunctions.euler2T(initial_pose["position"], initial_pose["orientation"], extrinsic=False) 

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
                    T_rot = mathfunctions.euler2T( np.array([0.0,0.0,0.0], dtype="float"), 
                                                    ScyRot.from_euler('XYZ', [angles_tmp[0], angles_tmp[1], angles_tmp[2]], 
                                                                                degrees=True).as_matrix() ) 
                    
                    # Rotate around robot flange: T_b_f * T_f_m * T_rot * T_m_f
                    T_b_f_rot = np.linalg.multi_dot( [T_init, T_f_m, T_rot, np.linalg.inv(T_f_m)] ) 
                    T_b_f_rot_list.append(T_b_f_rot) 
        
        return T_b_f_rot_list 
    
    
    def get_eval_poses(self, initial_pose): 
        """ 
        Creates a list of robot endeffector motion poses for evaluation. 
        Strong rotations around the direction towards the laser tracker. 
        
        Input 
        - initial_pose [dict]:  is used as the center point for all calibration motions 
                                initial_pose["position"] = [x, y, z], initial_pose["orientation"] = [rx, ry, rz, rw]; 
                                if pose["orientation"] has only 3 elemens, then "XYZ" euler angles are assumed;  

        """  
        # check orientation type of initial pose
        if(len(initial_pose["orientation"]) == 3):
            # "XYZ" euler to quaternion 
            initial_pose["orientation"] = ScyRot.from_euler('XYZ', initial_pose["orientation"], degrees=True).as_quat() 
        self.initial_calibration_pose = initial_pose

        # create calibration poses 
        pose_list = []  
        # translation stays the same for all poses 
        pose = {"position":initial_pose["position"].copy()} 
        # euler rotations around each axis relative to the initial pose; 0.0 is automatically included 
        angle_list = [0.0, 20.0, 40.0, 60.0] 

        # create poses
        # around z-axis 
        for rz in angle_list: 
            # current xyz-euler angles relative initial pose  
            angles_tmp = [0.0, 0.0, rz] 
            # euler angles (rx, ry, rz) to quaternions (x, y, z, w); extrinsic rotation; 
            rot_tmp_quat = ScyRot.from_euler('xyz', angles_tmp, degrees=True).as_quat() 
            # rotate extrinsically relative to the initial pose by pre-multiplying 
            rot_result_quat = tf.transformations.quaternion_multiply(rot_tmp_quat, np.array(initial_pose["orientation"].copy())) 
            pose["orientation"] = rot_result_quat
            pose_list.append(pose.copy()) 
                    
        return pose_list


    def move_for_calibration(self, initial_pose, campairs=[0], warmup=True, offsetpoint=[0.0, 0.0, 0.0], angles=None, 
                             ExposureAutoReference=60, ExposureTime=150000.0):
        """
        Movement profile for automatically creating 27 images to calibrate the cameras with.
        
        Input 
        - initial_pose [dict]:  is used as the center point for all calibration motions 
                                initial_pose["position"] = [x, y, z], initial_pose["orientation"] = [rx, ry, rz, rw]; 
                                if pose["orientation"] has only 3 elemens, then "XYZ" euler angles are assumed;  
        - offsetpoint (3x1-list; float): rotate around this offset point; in m; default: no offset 
        - angles (3xn float list):  rotation angle lists for xyz axis in Euler. 
                                    Default is +-10° around every axis. 
                                    [[x angles], [y angles], [z angles]] 
        - campairs [list, int]:         Index list of camera pairs. From all camera pairs defined in the list, images are acquired. 
    
        """

        # get pose list 
        pose_list = self.get_calib_poses(initial_pose, offsetpoint=offsetpoint, angles=angles)
        
        # move to initial calibration pose first 
        self.move_robot(initial_pose, extrinsic_euler=False)  

        # # run camera in empty mode to warm it up 
        # for a in range(10): 
        #     self.cam_streamer.acquire_image_set(rootpath="", imgnum=a)

        # move robot to calibration poses 
        for i in range(len(pose_list)):
            # Move to first point automatically
            motion_successful = self.move_robot(pose_list[i])  
            if(motion_successful==False): 
                print("[CAMCALIB]["+str(campairs[0])+"] Robot could not reach pose. Skipping Pose.") 
                continue 
            # wait until the brightness adjusts 
            print("[CAMCALIB]["+str(campairs[0])+"] Pose "+str(i+1)+"/"+str(len(pose_list))+"") 
            print("[CAMCALIB]["+str(campairs[0])+"] Current robot pose:")
            self.get_pose(print_msg=True)

            # acquire images 
            self.tracker.get_imgs(self.tracker.camcalib_rootpath, i, campairs=campairs, warmup=warmup, 
                                  ExposureAutoReference=ExposureAutoReference, ExposureTime=ExposureTime) 
    
            print("")
       
            
    def move_for_calibration_multicam(self, initial_poses, warmup=True, offsetpoint=[0.0, 0.0, 0.0], angles=None, 
                                      ExposureAutoReference=60, ExposureTime=150000.0):
        """
        Wrapper for move_for_calibration to support multiple camera pairs. 
        
        Input:
        - initial_poses:    is a list of initial_pose from move_for_calibration
        """
        # clean directory 
        data_manager.delete_dircontent(self.tracker.camcalib_rootpath) 
        
        # loop through all camera pairs 
        for campair_idx in range(len(self.tracker.cam_streamer.campair_ips)):  
            self.move_for_calibration(initial_poses[campair_idx], campairs=[campair_idx], 
                                      warmup=warmup, offsetpoint=offsetpoint, angles=angles, 
                                      ExposureAutoReference=ExposureAutoReference, ExposureTime=ExposureTime) 

    
    def attach2gripper(self, waiting_period=25): 
        """
        Routine to half-automatically attach objects to the gripper. 
        Gripper opens and waits for "waiting_period" (default=20) seconds and then closes. 

        Input: 
        - robot (MoveObject):       Fully configured robot object.   
        - waiting_period (float):   Waiting time before closing the gripper. 
        """
        # decide if attached object should be changed. 
        change = False 
        opening_period = 0 
        # get which object is currently attached {'nothing', 'calibplate', 'sample'}
        attached_obj = self.config["attached_obj"]["value"] 
        # get the object that should be attached 
        target_obj = ""
        if (self.config["collect_calib_data"]["value"] == True): 
            target_obj = "calibplate" 
        else: 
            target_obj = "sample" 
        # check if the attached object is not the desired object 
        if(attached_obj != target_obj): 
            change = True # should be changed 

        if(change): 
            # waiting period before opening the gripper if another obj is already attached 
            if(attached_obj != "nothing"): 
                opening_period = 15 
            # change gripper 
            input("[GRIPPER] Press ENTER to start half-automatic attaching routine (wait-open-wait-close).")
            print("[GRIPPER] Starting attaching routine.") 
            print("[GRIPPER] In "+str(opening_period)+" seconds the gripper will open.") 
            time.sleep(opening_period) 
            self.gripper.open() 
            print("[GRIPPER] You have "+str(waiting_period)+" seconds to attach the object.") 
            time.sleep(waiting_period) 
            self.gripper.close() 
            # update attached object in config file 
            robot_tools.update_config({"attached_obj":target_obj}) 


    def test_motionpattern(self, motionpattern, cycles=1, stopping=False, tracker=False): 
        """
        Moves the robot to the given patterns to test the poses. 

        Input: 
        - motionpattern (list of 4x4 np float): List of 4x4 homogeneous transformation matrices. 
                                                Poses for robot flange in m. 
        - cycles (int):                         Number of times the set of motions should be repeated. 
        - stopping (bool):                      If True: stops after every motion and waits for the user to press ENTER. 
        - tracker (bool):                       If True: estimates sample pose with cameras for every motion. 

        Output (all in a list): 
        - successful_motions (list, int):       Index of motion patterns in motionpattern that were successful. 
        - unsuccessful_motions (list, int):     Index of motion patterns in motionpattern that were unsuccessful. 
        - successful_tracker (list, int):       Index of pose estimations in motionpattern that were successful. 
        - unsuccessful_tracker (list, int):     Index of pose estimations in motionpattern that were unsuccessful. 
        
        """
        # time.sleep(10)
        for i in range(cycles): 
            successful_motions = [] 
            unsuccessful_motions = [] 
            successful_tracker = [] 
            unsuccessful_tracker = [] 
            # motion pattern 1 test 
            for pattern1num in range(len(motionpattern)):
                if(stopping):
                    input("\n[MOTION_TEST] Press ENTER to start next motion from the list.\n") 
                motion_successful = self.move_robot(motionpattern[pattern1num]) 
                if(motion_successful):
                    motion_success_str = "successful."
                    successful_motions.append(pattern1num) 
                    if(tracker): 
                        try: 
                            R_c_s, t_c_s = self.tracker.get_sample_pose(impairnum=pattern1num) 
                            successful_tracker.append(pattern1num) 
                            tracker_success_str = "successful."
                        except Exception as error: 
                            unsuccessful_tracker.append(pattern1num) 
                            tracker_success_str = "NOT successful."
                else: 
                    motion_success_str = "NOT successful."
                    if(tracker): 
                        tracker_success_str = "NOT attempted."
                    unsuccessful_motions.append(pattern1num) 
                print("\n[MOTION_TEST] Motion "+str(pattern1num)+" was "+motion_success_str) 
                if(tracker): 
                    print("[MOTION_TEST] Tracking "+str(pattern1num)+" was "+tracker_success_str) 
                print("") 

        print("\n[MOTION_TEST] Finished.") 
        print("[MOTION_TEST] Motion successful: ", successful_motions) 
        print("[MOTION_TEST] Motion unsuccessful: ", unsuccessful_motions) 
        if(tracker): 
            print("[MOTION_TEST] Tracking successful: ", successful_tracker) 
            print("[MOTION_TEST] Tracking unsuccessful: ", unsuccessful_tracker) 
        print("")    
        return [successful_motions, unsuccessful_motions, successful_tracker, unsuccessful_tracker] 

    
    def save_camcalib_initialpose(self, campair_idx): 
        """TODO 
        Saves current robot pose into config file as initial calibration pose for camera pair with index idx. 
        """
        # get current pose of robot 
        robot_pose_current = self.get_pose(euler=True, extrinsic_euler=False) 
        # transform current robot pose into a dict 
        robot_pose_current_dict = {"position":robot_pose_current[:3].tolist(), "orientation":robot_pose_current[3:].tolist()} 
        # load config file with old initial poses for all camera pairs 
        config, config_path = robot_tools.load_config() 
        old_poses = config["initial_camcalib_pose"]["value"] 
        # update old poses with new pose for the desired camera pair campair_idx 
        old_poses[campair_idx] = robot_pose_current_dict 
        # save to config file 
        config_changes = {"initial_camcalib_pose":old_poses}  
        robot_tools.update_config(config_changes) 
        return old_poses 


if __name__ == "__main__":
    # # Initialize MoveObject connecting to our robot
    # stbl = MoveObject()

    # stbl.move_group.set_goal_orientation_tolerance(0.000001)
    # stbl.move_group.set_goal_position_tolerance(0.000001)

    print("")