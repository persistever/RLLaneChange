# coding:utf-8

from lane_change_env import Env
from RL_brain import DQN
from surrounding import Traffic
import numpy as np
import random


def run_task(env, no_gui, max_episode, net=None):
    step = 0
    for episode in range(max_episode):
        traffics_base = random.uniform(0.6, 0.8)
        traffics = Traffic(trafficBase=traffics_base, trafficList=None)
        done = False
        if (episode+1) % 50 == 0:
            observation = env.reset(nogui=False)
        else:
            observation = env.reset(nogui=no_gui)
        observation = np.array(observation)
        flag = 0
        while done is False:
            print('Episode '+str((episode+1)))
            print('Make decision '+str(step))
            action_high, action_low = net.choose_action(observation)
            print("action_high: " + str(action_high) + " action_low: " + str(action_low))
            observation_, reward, done, info = env.step(action_high=action_high, action_low=action_low)
            # if flag == 0:
            #     # action_high = 2
            #     # action_low = 4
            #     observation_, reward, done, info = env.step(action_high=action_high, action_low=action_low)
            #     flag = 1
            # else:
            #     # action_high = 0
            #     # action_low = 3
            #     observation_, reward, done, info = env.step(action_high=action_high, action_low=action_low)
            #     flag = 0
            # observation, done, reward = env.step(action_high=1, action_low=1)
            observation_ = np.array(observation_)
            net.store_transition(observation, action_high, action_low, reward, observation_)
            step += 1
            if step > 50 and step % 1 == 0:
                net.learn()
            observation = observation_
            print("reward: " + str(reward))
            print("info: "+str(info))
            print("-------------------------")


if __name__ == "__main__":
    traffics = Traffic(trafficBase=0.4, trafficList=None)
    LC_env = Env(ego_start_time=30)
    dqn = DQN(n_features=3, e_greedy_increment=0.001)
    run_task(env=LC_env, no_gui=True, max_episode=50, net=dqn)


