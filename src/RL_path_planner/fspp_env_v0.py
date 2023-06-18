import os
import time
import subprocess
import rospy
import numpy as np
from uav.uav import UAV
from geometry.geometry import Geometry
from geometry_msgs.msg import Vector3

import gym
from gym import spaces

mrs_env = os.environ.copy()
mrs_env['UAV_NAME'] = 'uav1'
mrs_env['RUN_TYPE'] = 'simulation'
mrs_env['UAV_TYPE'] = 'f450'
mrs_env['WORLD_NAME'] = 'simulation_local'
mrs_env['SENSORS'] = 'garmin_down'
mrs_env['ODOMETRY_TYPE'] = 'gps'
mrs_env['PX4_SIM_SPEED_FACTOR'] = '4'

class FSPPEnv(gym.Env):
    MAX_DISTANCE = 5.0 # [m] distância máxima do goal...
    N_HITS_RESET_GOAL = 200 # quantidade de episódios até resetar o goal
    POSSIBLE_GOALS = [
        [0.0, 2.0, 2.0],
        [-1.5, 1.4, 2.5],
        [-2.2, -0.8, 3.0],
        [0.9, 2.9, 2.0],
        [-4.0, 4.5, 2.5],
        [4.4, -4.3, 3.0],
        [2.2, -6.1, 3.0],
        [2.2, -6.8, 3.0],
        [9.4, 1.9, 2.5],
        [-8.0, 8.0, 2.5],
        [7.5, 9.5, 2.5],
        [4.6, -13.1, 3.0],
        [15.0, 7.0, 3.0],
        [13.2, 12.0, 2.5],
        [-22.0, 15.0, 2.5],
        [20.0, 20.0, 2.0],
        [-26.0, 16.0, 2.5],
        [-32.0, 30.0, 2.0]
    ]

    def __init__(self, uav_id=1) -> None:
        
        self.uav = UAV(uav_id=uav_id)

        self.action_space = spaces.Box(low=-1.5, high=1.5, shape=(3,), dtype=np.float32)

        self.observation_space = spaces.Dict({
            'goal_distance': spaces.Box(low=0.0, high=np.inf, shape=(1,), dtype=np.float32),
            'position': spaces.Box(low=-100.0, high=100.0, shape=(3,), dtype=np.float32),
            'obstacles': spaces.Box(low=0.0, high=1.0, shape=(720,), dtype=np.float32),
        })

        self.goal = None
        self.initial_distance_to_goal = None
        self.n_hits_on_taget = 0

    def step(self, action):
        velocity = Vector3(*action)
        uav_position = self.uav.uav_info.get_uav_position(tolist=True)
        self.uav.movements.apply_velocity(velocity)

        observation = self._get_observation()
        reward = self._calculate_reward(uav_position)
        done = self._check_episode_completion()
        info = {}

        # print(f'observation: {observation}, reward: {reward}, done: {done}, info: {info}, action: {action}')
        # print(f'reward: {reward}')
        # # print(*action)
        if np.nan in action:
            rospy.logerr('[FSPP.step]: NAN action')

        return observation, reward, done, info

    def reset(self):
        self._reset_mrs_nodes()

        # checar tempo maximo para takeoff
        self.uav.movements.takeoff()

        # gera um novo goal a cada self.N_HITS_RESET_GOAL (acerto) no target...
        if self.n_hits_on_taget % self.N_HITS_RESET_GOAL == 0:
            self.goal = self._generate_random_goal()

        self.initial_distance_to_goal = Geometry.euclidean_distance(
            [0.0, 0.0, 2.0], self.goal)
        
        rospy.loginfo('[FSPPEnv.reset]: env resetado')

        return self._get_observation()

    def _distance_to_goal(self):
        uav_position = self.uav.uav_info.get_uav_position(tolist=True)
        return Geometry.euclidean_distance(uav_position, self.goal)

    def _calculate_reward(self, prev_uav_position):
        reward = 0
        distance_to_goal = self._distance_to_goal()
        
        if self.uav.uav_info.get_active_tracker() == 'NullTracker': # bateu e caiu
            reward = -20.0
        elif self.uav.movements.in_target(self.goal): # chegou no alvo
            reward = 100.0
        else:
            prev_distance_to_goal = Geometry.euclidean_distance(
                prev_uav_position, self.goal)
            if distance_to_goal > prev_distance_to_goal: # se distanciou do goal
                reward = -10.0
            else:
                reward = (prev_distance_to_goal - distance_to_goal) * 10.0 # mais pontos cada vez que se aproxima do goal...

        return reward
    
    def _get_observation(self):
        laser_scan = self.uav.uav_info.get_laser_scan()
        ranges = np.array(laser_scan.ranges)
        ranges[np.isinf(ranges)] = laser_scan.range_max
        uav_position = self.uav.uav_info.get_uav_position(tolist=True)
        goal_distance = Geometry.euclidean_distance(uav_position, self.goal)
        observation = {
            'goal_distance': np.array(goal_distance),
            'position': np.array(uav_position),
            'obstacles': self._normalize(laser_scan.range_min, laser_scan.range_max, ranges),
        }
        return observation

    def _generate_random_goal(self):
        if self.n_hits_on_taget >= 2000:
            index = np.random.randint(len(self.POSSIBLE_GOALS))
            goal = self.POSSIBLE_GOALS[index]
        else:
            # pega um goal de acordo com o nível atual de dificuldade...
            difficulty_factor = self.n_hits_on_taget // self.N_HITS_RESET_GOAL
            index = min(difficulty_factor, len(self.POSSIBLE_GOALS) - 1)
            goal = self.POSSIBLE_GOALS[index]
        print(f'GOAL: {goal}')
        return goal

    def _check_episode_completion(self): # verificando se o episódio terminou...
        done = False
        distance_to_goal = self._distance_to_goal()

        if self.uav.uav_info.get_active_tracker() == 'NullTracker': # bateu e caiu
            done = True
            rospy.loginfo('[FSPPEnv._check_episode_completion]: bateu e caiu')
        elif self.uav.movements.in_target(self.goal): # chegou no destino
            done = True
            self.n_hits_on_taget += 1
            rospy.loginfo('[FSPPEnv._check_episode_completion]: chegou no destino')
        elif distance_to_goal > self.initial_distance_to_goal + self.MAX_DISTANCE: # se distanciou muito do goal
            done = True
            rospy.loginfo('[FSPPEnv._check_episode_completion]: se distanciou muito do goal')

        return done
    
    def _normalize(self, min_val: float, max_val: float, values: np.array):
        return (values - min_val) / (max_val - min_val)
    
    def _reset_mrs_nodes(self):
        self._kill_nodes()
        self._start_nodes()
    
    def _start_nodes(self):
        subprocess.Popen(
            'roslaunch mrs_simulation simulation.launch gui:=false world_name:=forest', 
            shell=True,
            env=mrs_env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT)
        time.sleep(5.0)
        subprocess.Popen(
            'rosservice call /mrs_drone_spawner/spawn "1 $UAV_TYPE --enable-rangefinder --enable-rplidar --pos 0 0 1 0"', 
            shell=True,
            env=mrs_env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT)
        subprocess.Popen(
            'roslaunch mrs_uav_general automatic_start.launch', 
            shell=True,
            env=mrs_env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT)
        time.sleep(10.0)
        subprocess.Popen(
            'roslaunch mrs_uav_general core.launch', 
            shell=True,
            env=mrs_env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT)
        time.sleep(5.0)

    def _kill_nodes(self):
        subprocess.call(
            ['sh', './kill.sh'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT)
        rospy.sleep(2.0)

    def render(self):
        ...
    
    def close(self):
        ...