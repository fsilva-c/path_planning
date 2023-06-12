#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
from uav.uav import UAV
from RL_path_planner.fspp_env_v1 import FSPPEnv
from stable_baselines3 import DQN
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback

uav = UAV(uav_id=1)

def start():
    subprocess.Popen(['roscore']) # inicia ros master...
    rospy.sleep(2.0)

    rospy.init_node('rl_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    
    env = DummyVecEnv([lambda: Monitor(FSPPEnv())])
    model = DQN(
        'MultiInputPolicy',
        env,
        verbose=1,
        # learning_rate=,
        # batch_size=,
        # train_freq=,
        # target_update_interval=,
        # learning_starts=,
        # buffer_size=,
        # max_grad_norm=,
        # exploration_fraction=,
        # exploration_final_eps=,
        device='cuda',
    )

    # callbacks = []
    # eval_callback = EvalCallback(
    #     env,
    #     callback_on_new_best=None,
    #     n_eval_episodes=5,
    #     best_model_save_path=".",
    #     log_path=".",
    #     eval_freq=10000,
    # )
    # callbacks.append(eval_callback)

    # kwargs = {}
    # kwargs["callback"] = callbacks

    model.learn(
        total_timesteps=1e5,
        # tb_log_name="dqn_drone_run_" + str(rospy.get_time()),
        # **kwargs
    )
    model.save(f'DQN_training_model_docs')

start()
