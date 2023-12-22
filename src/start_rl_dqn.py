#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
import subprocess
from uav_interface.uav import UAV
from RL_path_planner.fspp_env import FSPPEnv
from stable_baselines3 import DQN
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
from stable_baselines3.common.logger import configure

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

    total_steps = 1e8

    fspp_env = FSPPEnv(act_space='discrete')
    env = DummyVecEnv([lambda: Monitor(fspp_env)])

    # logger...
    new_logger = configure('DQN_fsppenv_log', ['stdout', 'csv'])

    # callback
    callback = CallbackList([CheckpointCallback(
        save_freq=100_000,
        name_prefix='DQN_model_test',
        save_path='DQN_models_test'
    )])

    model = DQN('MlpPolicy', env, verbose=1)
    # model = DQN.load('DQN_models_test/LAST_DQN_model_test_20100000_steps', env=env)
    
    if MODE == 'train':
        model.set_logger(new_logger)
        model.learn(total_timesteps=total_steps, callback=callback, reset_num_timesteps=False)
        model.save('UAV_NAV_DQN')
    else:
        obs = env.reset()
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, _ = env.step(action)
            if done:
                break

start()
