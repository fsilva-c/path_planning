#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
from uav.uav import UAV
from RL_path_planner.fspp_env_v0 import FSPPEnv
from stable_baselines3 import DQN
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import StopTrainingOnRewardThreshold, EvalCallback

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
    model = DQN(
        'MultiInputPolicy',
        env,
        verbose=1,
        device='cuda',
    )

    callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=200, verbose=1)
    eval_callback = EvalCallback(env, callback_on_new_best=callback_on_best)

    model.learn(total_timesteps=5e5, callback=eval_callback)
    model.save(f'DQN_training_model_docs')

start()
