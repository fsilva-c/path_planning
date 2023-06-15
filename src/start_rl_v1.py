#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
from uav.uav import UAV
from RL_path_planner.fspp_env_v1 import FSPPEnv
from stable_baselines3 import DDPG
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback

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
    model = DDPG(
        'MultiInputPolicy',
        env,
        verbose=1,
        device='cuda',
    )

    eval_callback = EvalCallback(
        env, 
        n_eval_episodes=5,
        best_model_save_path='.'
    )

    model.learn(total_timesteps=5e5, callback=eval_callback)
    model.save(f'DQN_training_model_DDPG')

start()
