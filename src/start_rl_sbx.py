#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
import subprocess
from uav_interface.uav import UAV
from RL_path_planner.fspp_env_v4_empty import FSPPEnv
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.logger import configure
from sbx import PPO

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

    env = FSPPEnv()

    new_logger = configure('pposbx_fsppenv_log', ['stdout', 'csv'])

    model = PPO(
        'MlpPolicy',
        env,
        verbose=1,
        batch_size=64,
    )                                                                  

    model.set_logger(new_logger)

    eval_callback = EvalCallback(
        env, 
        n_eval_episodes=5,
        best_model_save_path='.'
    )

    model.learn(total_timesteps=100_000, callback=eval_callback)
    model.save('training_model_UAV_SBX')

start()
