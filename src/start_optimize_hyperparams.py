#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import subprocess
import rospy
import optuna
from RL_path_planner.fspp_env_v0 import FSPPEnv
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor

def optimize_ppo(trial):
    return {
        'batch_size': trial.suggest_categorical('batch_size', [8, 16, 32, 64, 128, 256, 512]),
        'n_steps': trial.suggest_categorical('n_steps', [8, 16, 32, 64, 128, 256, 512, 1024, 2048]),
        'gamma': trial.suggest_float('gamma', 0.9, 0.9999),
        'learning_rate': trial.suggest_float('learning_rate', 1e-8, 1e-1),
        'ent_coef': trial.suggest_float('ent_coef', 1e-8, 1e-1),
        'n_epochs': trial.suggest_int('n_epochs', 3, 10),
        'gae_lambda': trial.suggest_float('gae_lambda', 0.8, 1.)
    }

def optimize_agent(trial):
    model_params = optimize_ppo(trial)
    env = DummyVecEnv([lambda: Monitor(FSPPEnv())])
    model = PPO('MultiInputPolicy', env, verbose=0, device='cuda', **model_params)
    model.learn(10000)
    mean_reward, _ = evaluate_policy(model, env, n_eval_episodes=10)
    return -1 * mean_reward

if __name__ == '__main__':
    print('Realizando tuning dos hyperparâmetros...')
    subprocess.Popen(
        'roscore',
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT) # inicia ros master...
    time.sleep(2.0)
    rospy.init_node('rl_tuning', anonymous=True)
    rospy.loginfo('Iniciando os testes...')
    study = optuna.create_study()
    try:
        study.optimize(optimize_agent, n_trials=100, n_jobs=1, show_progress_bar=True)
    except KeyboardInterrupt:
        print('Interrupted by keyboard.')
