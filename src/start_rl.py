#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from uav.uav import UAV
from RL_path_planner.fspp_env_v0 import FSPPEnv
from stable_baselines3 import DQN
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback

uav = UAV(uav_id=1)

import roslaunch

node = roslaunch.core.Node(
    'mrs_simulation', 
    'simulation.launch',
    remap_args=[('world_name', 'forest')]
)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
print(process.is_alive())
process.stop()

# def start_mrs():
#     os.environ['UAV_NAME'] = 'uav1'
#     os.environ['RUN_TYPE'] = 'simulation'
#     os.environ['UAV_TYPE'] = 'f450'
#     os.environ['WORLD_NAME'] = 'simulation_local'
#     os.environ['SENSORS'] = 'garmin_down'
#     os.environ['ODOMETRY_TYPE'] = 'gps'
#     os.environ['PX4_SIM_SPEED_FACTOR'] = '4'

#     commands = [
#         'roscore',
#         'waitForRos; roslaunch mrs_simulation simulation.launch gui:=false world_name:=forest',
#         'waitForSimulation; roslaunch mrs_uav_status status.launch',
#         'waitForSimulation; rosservice call /mrs_drone_spawner/spawn "1 $UAV_TYPE --enable-rangefinder --enable-rplidar --pos_file `pwd`/pos.yaml"',
#         'waitForOdometry; roslaunch mrs_uav_general core.launch config_uav_manager:=./custom_configs/uav_manager.yaml config_odometry:=./custom_configs/odometry.yaml',
#         'waitForSimulation; roslaunch mrs_uav_general automatic_start.launch',
#         'waitForControl; rosservice call /$UAV_NAME/mavros/cmd/arming 1; sleep 2; rosservice call /$UAV_NAME/mavros/set_mode 0 offboard',
#     ]

#     for command in commands:
#         subprocess.call(command, shell=True)

# def kill_mrs():
#     commands_to_kill = [
#         'waitForRos; roslaunch mrs_simulation simulation.launch gui:=false world_name:=forest',
#         'waitForSimulation; roslaunch mrs_uav_status status.launch',
#         'waitForSimulation; rosservice call /mrs_drone_spawner/spawn "1 $UAV_TYPE --enable-rangefinder --enable-rplidar --pos_file `pwd`/pos.yaml"',
#         'waitForOdometry; roslaunch mrs_uav_general core.launch config_uav_manager:=./custom_configs/uav_manager.yaml config_odometry:=./custom_configs/odometry.yaml',
#         'waitForSimulation; roslaunch mrs_uav_general automatic_start.launch',
#     ]
    
#     for command in commands_to_kill:
#         subprocess.call(f'pkill -f "{command}"', shell=True)
    

#     for command in commands_to_kill:
#         subprocess.call(command, shell=True)

def start():
    rospy.init_node('rl_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    # uav.movements.takeoff()
    while rospy.get_time() <= 30.0:
        rospy.sleep(0.1)

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
        total_timesteps=10000,
        # tb_log_name="dqn_drone_run_" + str(rospy.get_time()),
        # **kwargs
    )
    # model.save(f'DQN_training_model_docs')

# start()
