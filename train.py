# coding:utf-8

from lane_change_env import Env


def run_task(env, max_episode):
    step = 0
    for episode in range(max_episode):
        done = False
        observation = env.reset(nogui=True)
        while done is False:
            done = env.step(action=[1, None])
            step += 1


if __name__ == "__main__":
    env = Env(ego_start_time=30)
    run_task(env, 10)


