from view_plan.create_env import *
from view_plan.action_moveRobot import *

def viewPlan_RL(box_attacher):
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
            n_state, reward, done, info = env.step(box_attacher, action)
            score += reward
        # print(n_state, reward, done, info, score)
        print('Episode:{} Score:{}'.format(episode, score))