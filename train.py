# coding:utf-8

from lane_change_env import Env
from RL_brain import DQN


def run_task(env, max_episode):
    step = 0
    for episode in range(max_episode):
        done = False
        observation = env.reset(nogui=False)
        flag = 0
        while done is False:
            # action_high, action_low = DQN.choose_action(obsevation[0], observation[1], observation[2], observation[3])
            # ovservation_, reward, done = env.step(action=[action_high, action_low])
            # if flag == 0:
            #     observation, done = env.step(action_high=0, action_low=1)
            #     flag = 1
            # else:
            #     observation, done = env.step(action_high=2, action_low=1)
            #     flag = 0
            observation, done = env.step(action_high=1, action_low=1)
            step += 1


if __name__ == "__main__":
    env = Env(ego_start_time=30)
    # dqn = DQN(n_features=2)
    run_task(env, 10)


