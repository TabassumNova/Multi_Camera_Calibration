from gym import Env
from gym.spaces import Discrete, Box, MultiBinary, Dict
import numpy as np
import random
import src.multical.app.boards as boards
import os
from .action_moveRobot import *
from src.aravis_image_acquisition import *

board_path = "/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/view_plan/boards.yaml"
train_path = "/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/view_plan/train/"

class ShowerEnv(Env):
    def __init__(self):
        # Actions we can take: position (x,y,z), orientation(x,y,z)
        self.action_space = Dict({"move_x": Discrete(3),
        "move_y": Discrete(3), 
        "move_z": Discrete(3), 
        "rotate_x": Discrete(3), 
        "rotate_y": Discrete(3), 
        "rotate_z": Discrete(3)})

        # Checker detection array
        # self.observation_space = Box(low=np.array([0]), high=np.array([88]))
        b = Box(low=np.array([-180]), high=np.array([180]))
        self.observation_space = Dict({"08320217_detection": Box(low=np.array([0]), high=np.array([88])),
                                       "08320218_detection": Box(low=np.array([0]), high=np.array([88])),
                                       "08320220_detection": Box(low=np.array([0]), high=np.array([88])),
                                       "08320221_detection": Box(low=np.array([0]), high=np.array([88])),
                                       "08320222_detection": Box(low=np.array([0]), high=np.array([88])),
                                       "36220113_detection": Box(low=np.array([0]), high=np.array([88])),
                                       })
        # Set start temp
        # self.state = 38 + random.randint(-3, 3)
        self.pose = 1
        self.detection_dict = {}
        camera_serial = arv_get_image(train_path, self.pose)
        self.state = self.detection(board_path, train_path)
        self.pose += 1
        # Set shower length
        self.shower_length = 60

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
        # Apply action
        # 0 -1 = -1 temperature
        # 1 -1 = 0
        # 2 -1 = 1 temperature
        moveRobot(box_attacher, action)
        camera_serial = arv_get_image(train_path, self.pose)
        self.state = self.detection(board_path, train_path)
        self.pose += 1
        # self.state += action - 1
        # # Reduce shower length by 1 second
        # self.shower_length -= 1

        # Calculate reward
        reward = self.reward()
        # if self.state >= 37 and self.state <= 39:
        #     reward = 1
        # else:
        #     reward = -1

            # Check if shower is done
        if self.shower_length <= 0:
            done = True
        else:
            done = False

        # Apply temperature noise
        # self.state += random.randint(-1,1)
        # Set placeholder for info
        info = {}

        # Return step information
        return self.state, reward, done, info

    def reward():
        if self.detection_dict[self.pose]['total_detection'] > self.detection_dict[self.pose-1]['total_detection']:
            reward = 1
        else:
            reward = -1
        pass

    def render(self):
        # Implement viz
        pass

    def reset(self):
        # Reset shower temperature
        self.pose = 1
        self.detection_dict = {}
        self.state = self.detection(self, board_path, train_path)
        # Reset shower time
        self.shower_length = 60
        return self.state

