#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import rospy
import subprocess
from uav_interface.uav import UAV
from RL_path_planner.fspp_env_v1 import FSPPEnv
from stable_baselines3 import DQN
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.logger import configure

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

    env = DummyVecEnv([lambda: Monitor(FSPPEnv())])

    # logger...
    new_logger = configure('dqn_fsppenv_log', ['stdout', 'csv'])

    model = DQN(
        'MultiInputPolicy',
        env,
        verbose=1,
        device='cuda',
        stats_window_size=1,
    ) if 'best_model_DQN.zip' not in os.listdir('.') else DQN.load('best_model_DQN.zip', env=env)

    model.set_logger(new_logger)

    eval_callback = EvalCallback(
        env, 
        n_eval_episodes=5,
        best_model_save_path='.'
    )

    model.learn(total_timesteps=2e7, callback=eval_callback)
    model.save('training_model_UAV_DQN')

start()
