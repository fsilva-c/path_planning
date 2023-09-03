#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
from uuid import uuid4
from uav.uav import UAV
from RL_path_planner.fspp_env_v0 import FSPPEnv
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.logger import configure

uav = UAV(uav_id=1)

def start():
    subprocess.Popen(
        ['roscore'], 
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT) # inicia ros master...
    rospy.sleep(2.0)

    rospy.init_node('rl_mission', anonymous=True)
    rospy.loginfo('Iniciando os testes...')

    env = DummyVecEnv([lambda: Monitor(FSPPEnv())])

    instance = str(uuid4().hex[:10])

    # logger...
    new_logger = configure(f'ppo_fsppenv_log_{instance}', ['stdout', 'csv'])

    model = PPO(
        'MultiInputPolicy',
        env,
        verbose=1,
        device='cuda',
        batch_size=128,
        n_steps=2048,
        learning_rate=0.00025,
        gamma=0.95,
        stats_window_size=1, # estatísticas do PPO
    )

    model.set_logger(new_logger)

    eval_callback = EvalCallback(
        env, 
        n_eval_episodes=5,
        best_model_save_path=instance
    )

    model.learn(total_timesteps=2e5, callback=eval_callback)
    model.save(f'training_model_UAV_PPO_{instance}')

start()
