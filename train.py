# coding:utf-8

from lane_change_env import Env
from RL_brain import DQN


def run_task(env, max_episode, net):
    step = 0
    for episode in range(max_episode):
        done = False
        observation = env.reset(nogui=False)
        flag = 0
        while done is False:
            print('Make decision '+str(step))
            # action_high, action_low = DQN.choose_action(observation)
            # ovservation_, reward, done = env.step(action=[action_high, action_low])
            if flag == 0:
                action_high = 2
                action_low = 4
                observation_, done, reward, info = env.step(action_high=action_high, action_low=action_low)
                flag = 1
            else:
                action_high = 0
                action_low = 3
                observation_, done, reward, info = env.step(action_high=action_high, action_low=action_low)
                flag = 0
            # observation, done, reward = env.step(action_high=1, action_low=1)
            net.store_transition(observation, action_high, action_low, observation_)
            step += 1
            observation = observation_
            print("info: "+str(info))
            print("reward: " + str(reward))
            print("observation: "+str(observation))
            print("-------------------------")


if __name__ == "__main__":
    LC_env = Env(ego_start_time=30)
    dqn = DQN(n_features=3)
    run_task(LC_env, 1, dqn)


