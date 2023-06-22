from gym import Env
from gym.spaces import Discrete, Box, MultiBinary, Dict
import numpy as np
import random
import src.multical.app.boards as boards
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

from .action_moveRobot import *
from src.aravis_image_acquisition import *

board_path = "/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/view_plan/boards.yaml"
train_path = "/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/view_plan/train/"

class ViewPlanEnv(Env):
    def __init__(self, box_attacher):
        # Actions we can take: position (x,y,z), orientation(x,y,z)
        self.action_space = Dict({"move_x": Discrete(3),
        "move_y": Discrete(3), 
        "move_z": Discrete(3), 
        "rotate_x": Discrete(3), 
        "rotate_y": Discrete(3), 
        "rotate_z": Discrete(3)})

        b = Box(low=np.array([-180]), high=np.array([180]))
        self.observation_space = Dict({"08320217_detection": Box(low=np.array([0]), high=np.array([88])),
                                       "08320218_detection": Box(low=np.array([0]), high=np.array([88])),
                                       "08320220_detection": Box(low=np.array([0]), high=np.array([88])),
                                       "08320221_detection": Box(low=np.array([0]), high=np.array([88])),
                                       "08320222_detection": Box(low=np.array([0]), high=np.array([88])),
                                       "36220113_detection": Box(low=np.array([0]), high=np.array([88])),
                                       })
        
        self.box_attacher = box_attacher
        self.workspace = None
        self.json_dict = {}
        enter = input("Hit ENTER if you want to start planning: ")
        self.reset_position()
        self.initial_pose = get_pose(self.box_attacher, euler=True)

        self.pose = 1
        self.detection_dict = {}
        self.camera_serial = arv_get_image(train_path, self.pose)
        self.initiate_workspace()

        # self.state = self.detection(board_path, train_path)
        self.state = self.workspace.pose_table
        # self.pose += 1
        self.plan_length = 60

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

    def detection(self, board_path, train_path):
        self.detection_dict[self.pose] = {}
        total_detect = 0
        for subdir, dirs, files in os.walk(train_path):
            corners = 0
            for camera in dirs:
                detect_img = os.path.join(train_path, camera, 'p' + str(self.pose) + '.png')
                b = boards.Boards(boards=board_path, detect=detect_img, pixels_mm=10, show_image=False)
                detection = b.execute()
                for d in detection:
                    corners += d.ids.size
                key = str(camera) + "_detection"
                self.detection_dict[self.pose][key] = corners
                total_detect += corners
            self.detection_dict[self.pose]["total_detection"] = total_detect
        return self.detection_dict[self.pose]

    def step(self, box_attacher, action):

        moveRobot(box_attacher, action)
        self.pose += 1
        self.camera_serial = arv_get_image(train_path, self.pose)
        self.initiate_workspace()
        self.state = self.workspace.pose_table
        # self.pose += 1

        # Calculate reward
        reward = self.reward()

        if self.plan_length <= 0:
            done = True
        else:
            done = False

        info = {}

        # Return step information
        return self.state, reward, done, info

    def reward(self):
        if self.detection_dict[self.pose-1] == 0:
            reward = -1
            self.reset()
        elif self.detection_dict[self.pose-1]['total_detection'] > self.detection_dict[self.pose-2]['total_detection']:
            reward = 1
        else:
            reward = -1

        self.check_reprojection_error()

        pass
        return reward

    def render(self):
        # Implement viz
        pass

    def load_camera_intrinsic(self):
        pass

    def check_reprojection_error(self):
        pass

    def check_board_angle():
        pass

    def check_dataset_variety():
        pass  

    def reset(self):
        common_focus = [-61, -19, 117, 112, 5, -247]
        plan = self.box_attacher.move_robot_joints(np.array(common_focus))
        
        pass
        '''
        self.pose = 1
        self.detection_dict = {}
        self.state = self.detection(board_path, train_path)
        self.plan_length = 60
        return self.state
        '''

