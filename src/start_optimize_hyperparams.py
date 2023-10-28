#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import subprocess
import rospy
import optuna
from RL_path_planner.fspp_env_v3 import FSPPEnv
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback

def optimize_ppo(trial):
    return {
        'batch_size': trial.suggest_categorical('batch_size', [32, 64, 128, 256]),
        'n_steps': trial.suggest_categorical('n_steps', [64, 128, 256, 512, 1024, 2048]),
        'gamma': trial.suggest_float('gamma', 0.9, 0.9999),
        'learning_rate': trial.suggest_float('learning_rate', 1e-5, 1e-3, log=True),
        'ent_coef': trial.suggest_float('ent_coef', 1e-5, 1e-3, log=True),
        'n_epochs': trial.suggest_int('n_epochs', 3, 10),
        'gae_lambda': trial.suggest_float('gae_lambda', 0.8, 1.0)
    }

def optimize_agent(trial):
    try:
        model_params = optimize_ppo(trial)
        env = DummyVecEnv([lambda: Monitor(FSPPEnv())])
        model = PPO('MlpPolicy', env, verbose=0, device='cuda', **model_params)
        
        eval_callback = EvalCallback(env, best_model_save_path='.', log_path='.', eval_freq=1000, deterministic=True, render=False)
        model.learn(total_timesteps=100000, callback=eval_callback)
        
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
