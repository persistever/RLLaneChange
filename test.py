# coding:utf-8

from lane_change_env import Env
from RL_brain import DQN
from surrounding import Traffic
import numpy as np
import random
import time


def run_task(env, no_gui, max_episode, net=None):
    start = time.clock()
    step = 0
    traffics_base = random.uniform(0.7, 0.9)
    for episode in range(max_episode):
        traffics = Traffic(trafficBase=traffics_base, trafficList=None)
        done = False
        observation = env.reset(nogui=no_gui)
        observation = np.array(observation)
        # flag = 0
        while done is False:
            print('Episode '+str((episode+1)))
            print('Make decision '+str(step))
            action_high, action_low = net.choose_action(observation)
            observation_, reward, done, info = env.test_step(action_high=action_high, action_low=action_low)
            # if flag == 0:
            #     action_high = 2
            #     action_low = 1
            #     observation_, reward, done, info = env.step(action_high=action_high, action_low=action_low)
            #     flag = 1
            # else:
            #     action_high = 0
            #     action_low = 1
            #     observation_, reward, done, info = env.step(action_high=action_high, action_low=action_low)
            #     flag = 0
            observation_ = np.array(observation_)
            print("action_high: " + str(action_high) + " action_low: " + str(action_low))
            print("reward: " + str(reward))
            print("info: "+str(info))
            print("-------------------------")
            if (step+1) % 100 == 0:
                elapsed = (time.clock() - start)
                print("Step: "+str(step+1)+" Time Used: "+str(elapsed))
            observation = observation_
            step += 1


if __name__ == "__main__":
    LC_env = Env(ego_start_time=100)
    # Just show
    dqn = DQN(n_features=6, e_greedy_start=1, e_greedy_increment=0.01, is_save=True, is_restore=True)
    run_task(env=LC_env, no_gui=False, max_episode=1, net=dqn)


