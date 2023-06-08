#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from uav.uav import UAV
from classic_path_planner.path_planner import PathPlanner

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

    env = DummyVecEnv([lambda: FSPPEnv()])
    model = PPO(
        'MlpPolicy', 
        env, 
        n_epochs=1000,
        verbose=1
    )
    # model = PPO.load('PPO_training_model')
    model.learn(total_timesteps=2e4)
    model.save('PPO_training_model')

# start_astar_mission()
start_rl_mission()
