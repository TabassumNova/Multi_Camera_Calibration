from gym import Env
from gym.spaces import Discrete, Box, MultiBinary, Dict
import numpy as np
import random
import multical.app.boards as boards
import os

def detection(pose):
    board_yaml = "D:\MY_DRIVE_N\Masters_thesis\Dataset/view_plan\V2/boards.yaml"
    detect_folder = "train folder"
    detection_dict = {}
    for subdir, dirs, files in os.walk(detect_folder):
        for camera in dirs:
            detect_img = os.path.join(detect_folder, camera, 'p' + str(pose) + '.png')
            b = boards.Boards(boards=board_yaml, detect=detect_img, pixels_mm=10, show_image=False)
            detection = b.execute()
            key = str(camera) + "_detection"
            detection_dict[key] = detection

    return detection_dict

class ShowerEnv(Env):
    def __init__(self):
        # Actions we can take: position (x,y,z), orientation(x,y,z)
        self.action_space = MultiBinary(6)
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
        pose = 1
        self.state = detection(pose)
        # Set shower length
        self.shower_length = 60

    def step(self, action):
        # Apply action
        # 0 -1 = -1 temperature
        # 1 -1 = 0
        # 2 -1 = 1 temperature
        sample = action
        self.state += action - 1
        # Reduce shower length by 1 second
        self.shower_length -= 1

        # Calculate reward
        if self.state >= 37 and self.state <= 39:
            reward = 1
        else:
            reward = -1

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

    def render(self):
        # Implement viz
        pass

    def reset(self):
        # Reset shower temperature
        self.state = 38 + random.randint(-3, 3)
        # Reset shower time
        self.shower_length = 60
        return self.state

env = ShowerEnv()
print(env.action_space.sample())
print(env.observation_space.sample())
episodes = 10
for episode in range(1, episodes + 1):
    state = env.reset()
    done = False
    score = 0

    while not done:
        # env.render()
        action = env.action_space.sample()
        move_robot()
        n_state, reward, done, info = env.step(action)
        score += reward
    # print(n_state, reward, done, info, score)
    print('Episode:{} Score:{}'.format(episode, score))