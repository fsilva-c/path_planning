#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
import subprocess
from uav_interface.uav import UAV
from RL_path_planner.fspp_env_v0 import FSPPEnv
from stable_baselines3 import PPO
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
    new_logger = configure(f'ppo_fsppenv_log', ['stdout', 'csv'])

    model_params = {
        'n_steps': 1024,
        'batch_size': 32,
        'gamma': 0.9953295022725661, 
        'learning_rate': 0.000990097133814429, 
        'ent_coef': 0.0010103559410986254, 
        'n_epochs': 10, 
        'gae_lambda': 0.9191725806525997
    }

    model = PPO(
        'MultiInputPolicy',
        env,
        verbose=1,
        device='cuda',
        stats_window_size=1, # estatísticas do PPO
        **model_params
    )

    model.set_logger(new_logger)

    eval_callback = EvalCallback(
        env, 
        n_eval_episodes=5,
        best_model_save_path='.'
    )

    model.learn(total_timesteps=2e5, callback=eval_callback)
    model.save(f'training_model_UAV_PPO')

start()
