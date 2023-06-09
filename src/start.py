#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from uav.uav import UAV
from classic_path_planner.path_planner import PathPlanner
import os
from RL_path_planner.fspp_env_v0 import FSPPEnv
from stable_baselines3 import PPO, DQN
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy

uav = UAV(uav_id=1)
pp = PathPlanner(goal=(-26, 16, 2.5))

def start_astar_mission():
    rospy.init_node('astar_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    rospy.loginfo('Iniciando os testes...')
    pp.run()

def start_rl_mission():
    rospy.init_node('rl_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    rospy.loginfo('Iniciando os testes...')
    from RL_path_planner.fspp_env_v0 import FSPPEnv
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
    
    TRAIN = True
    env = DummyVecEnv([lambda: FSPPEnv()])
    
    if TRAIN:
        TIMESTEPS = 10000
        iters = 0
        model = PPO('MultiInputPolicy', env, verbose=1)
        while True:
            iters += 1
            model.learn(total_timesteps=1000, reset_num_timesteps=False)
            model.save(f'PPO_training_model_{TIMESTEPS * iters}')
    
    # model = PPO.load('PPO_training_model')
    # observation = env.reset()
    # done = False
    # while not done:
    #     action, state = model.predict(observation)
    #     observation, reward, done, info = env.step(action)
    #     print(f'action: {action}, observation: {observation}, reward: {reward}, done: {done}, info: {info}')

        # if done:
        #     env.reset()

def start_rl_mission_docs():
    rospy.init_node('rl_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    rospy.loginfo('Iniciando os testes...')

    models_dir = "models/PPO"
    logdir = "logs"

    if not os.path.exists(models_dir):
        os.makedirs(models_dir)

    if not os.path.exists(logdir):
        os.makedirs(logdir)

    env = DummyVecEnv([lambda: FSPPEnv()])
    model = PPO('MultiInputPolicy', env, verbose=1)
    model.learn(total_timesteps=1e5)
    model.save(f'{models_dir}/PPO_training_model_docs')
    del model
    model = PPO.load(f'{models_dir}/PPO_training_model_docs', env=env)
    mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=10)
    print(mean_reward, std_reward)
    vec_env = model.get_env()
    obs = vec_env.reset()
    for i in range(1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, rewards, dones, info = vec_env.step(action)
        print(obs, rewards, dones, info)

# start_astar_mission()
# start_rl_mission()
start_rl_mission_docs()
