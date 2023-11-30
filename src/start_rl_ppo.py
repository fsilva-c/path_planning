#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
import subprocess
from uav_interface.uav import UAV
from RL_path_planner.fspp_env_v3 import FSPPEnv
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.env_util import make_vec_env

MODE = 'train'
uav = UAV(uav_id=1)

def start():
    subprocess.Popen(
        'roscore',
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT) # inicia ros master...
    
    time.sleep(2.0)

    rospy.init_node('rl_mission', anonymous=True)
    rospy.loginfo('Iniciando os testes...')

    total_steps = 50000000

    env = make_vec_env(FSPPEnv)

    # logger...
    new_logger = configure('PPO_fsppenv_log', ['stdout', 'csv'])

    # callback
    callback = CallbackList([CheckpointCallback(
        save_freq=100_000,
        name_prefix='PPO_model_test',
        save_path='PPO_models_test'
    )])

    model = PPO(
        'MultiInputPolicy',
        env,
        verbose=0,
    )

    # model = PPO.load('DQN_models_test/LAST_DQN_model_test_18400000_steps', env=env)

    if MODE == 'train':
        model.set_logger(new_logger)
        model.learn(total_timesteps=total_steps, callback=callback)
        model.save('training_model_UAV_PPO')
    else:
        obs = env.reset()
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, _ = env.step(action)
            if done:
                break

start()
