#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
import subprocess
import torch as th
from uav_interface.uav import UAV
from RL_path_planner.fspp_env_v3 import FSPPEnv
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
    # env = FSPPEnv()

    # logger...
    new_logger = configure('ppo_fsppenv_log', ['stdout', 'csv'])

    model = PPO(
        'MlpPolicy',
        env,
        verbose=1,
        device='cuda',

        ent_coef=0.01,
        policy_kwargs = dict(
            net_arch=dict(pi=[256, 256], vf=[256, 256]),
            ortho_init=False,
            activation_fn=th.nn.ReLU
        )
    )

    model.set_logger(new_logger)

    eval_callback = EvalCallback(
        env, 
        n_eval_episodes=5,
        best_model_save_path='.'
    )

    # model.learn(total_timesteps=2e7, callback=eval_callback)
    model.learn(total_timesteps=100_000, callback=eval_callback)
    # model.save('training_model_UAV_PPO')

start()
