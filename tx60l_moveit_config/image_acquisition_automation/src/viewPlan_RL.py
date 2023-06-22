from .view_plan.create_env import *
from .view_plan.action_moveRobot import *

import numpy as np
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Flatten
from tensorflow.keras.optimizers import Adam

from rl.agents import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory

def viewPlan_RL(box_attacher):
    # common_focus = [-61, -19, 117, 112, 5, -247]
    # plan = box_attacher.move_robot_joints(np.array(common_focus))
    env = ViewPlanEnv()
    print(env.action_space.sample())
    print(env.observation_space.sample())
    episodes = 10
    for episode in range(1, episodes + 1):
        # state = env.reset()
        done = False
        score = 0

        while not done:
            # env.render()
            action = env.action_space.sample()
            n_state, reward, done, info = env.step(box_attacher, action)
            score += reward
        # print(n_state, reward, done, info, score)
        print('Episode:{} Score:{}'.format(episode, score))

def viewPlan_RL_model(box_attacher):
    env = ViewPlanEnv()
    states = env.observation_space.shape
    actions = env.action_space.n
    model = build_model(states, actions)

    dqn = build_agent(model, actions)
    dqn.compile(Adam(lr=1e-3), metrics=['mae'])
    dqn.fit(env, nb_steps=50000, visualize=False, verbose=1)

    scores = dqn.test(env, nb_episodes=100, visualize=False)
    print(np.mean(scores.history['episode_reward']))
    pass

def build_model(states, actions):
    model = Sequential()
    model.add(Dense(24, activation='relu', input_shape=states))
    model.add(Dense(24, activation='relu'))
    model.add(Dense(actions, activation='linear'))
    return model

def build_agent(model, actions):
    policy = BoltzmannQPolicy()
    memory = SequentialMemory(limit=50000, window_length=1)
    dqn = DQNAgent(model=model, memory=memory, policy=policy,
                  nb_actions=actions, nb_steps_warmup=10, target_model_update=1e-2)
    return dqn