#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import subprocess
import rospy
import optuna
from RL_path_planner.fspp_env_v0 import FSPPEnv
from stable_baselines3 import DQN
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback

def optimize_ppo(trial):
    return {
        'learning_rate': trial.suggest_float('learning_rate', 1e-4, 1e-2, log=True),
        'batch_size': trial.suggest_categorical('batch_size', [32, 64, 128, 256]),
        'gamma': trial.suggest_float('gamma', 0.9, 0.9999),
    }

def optimize_agent(trial):
    try:
        model_params = optimize_ppo(trial)
        env = DummyVecEnv([lambda: Monitor(FSPPEnv())])
        model = DQN('MultiInputPolicy', env, verbose=0, device='cuda', **model_params)
        
        eval_callback = EvalCallback(env, best_model_save_path='.', log_path='.', eval_freq=1000, deterministic=True, render=False)
        model.learn(total_timesteps=50000, callback=eval_callback)
        
        mean_reward, _ = evaluate_policy(model, env, n_eval_episodes=50)
        return -1 * mean_reward
    except Exception as e:
        print(f'Trial falhou devido a um erro: {e}')
        return optuna.TrialPruned
    
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
        study.optimize(optimize_agent, n_trials=100, timeout=600)
    except KeyboardInterrupt:
        print('Interrupted by keyboard.')
