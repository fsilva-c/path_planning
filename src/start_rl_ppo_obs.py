#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
import subprocess
from uav_interface.uav import UAV
from RL_path_planner.fspp_env_v3 import FSPPEnv
from stable_baselines3 import DQN
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
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

    # callback
    callback = CallbackList([CheckpointCallback(
        save_freq=10000,
        name_prefix='DQN_model_test',
        save_path='DQN_models_test'
    )])

    model = DQN(
        'MlpPolicy',
        env,
        verbose=1,
        train_freq=16,
        gradient_steps=8,
        gamma=0.99,
        exploration_fraction=0.2,
        exploration_final_eps=0.07,
        target_update_interval=600,
        learning_starts=1000,
        buffer_size=100000,
        batch_size=128,
        learning_rate=4e-3,
        policy_kwargs=dict(net_arch=[256, 256]),
        seed=2,
    )

    model.set_logger(new_logger)

    model.learn(total_timesteps=2e6, callback=callback)
    model.save('training_model_UAV_DQN')

start()
