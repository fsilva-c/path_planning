import os
import random
import time
import subprocess
import rospy
import pathlib
import numpy as np
from uav_interface.uav import UAV
from RL_path_planner.ros_waiter import ROSWaiter
from geometry.geometry import Geometry
from geometry_msgs.msg import Vector3

import gym
from gym.utils import seeding
from gym import spaces

mrs_env = os.environ.copy()
mrs_env['UAV_NAME'] = 'uav1'
mrs_env['RUN_TYPE'] = 'simulation'
mrs_env['UAV_TYPE'] = 'f450'
mrs_env['WORLD_NAME'] = 'simulation_local'
mrs_env['SENSORS'] = 'garmin_down'
mrs_env['ODOMETRY_TYPE'] = 'gps'
mrs_env['PX4_SIM_SPEED_FACTOR'] = '4'

filepath = pathlib.Path(__file__).resolve().parent
worlds_dir = filepath.parent.parent

ros_macros = os.path.join(filepath.parent, 'ros_macros.sh')
subprocess.Popen(f'bash -c "source {ros_macros}"', shell=True)

class FSPPEnv(gym.Env):
    MAX_DISTANCE = 4.0 # [m] distância máxima do goal...
    N_HITS_RESET_GOAL = 200 # quantidade de acertos até resetar o goal
    N_SCENARIOS = 3
    POSSIBLE_GOALS = [
        [0.0, -2.0, 1.2],
        [2.0, 0.0, 1.2],
        [-1.5, 1.4, 2.0],
        [0.9, 2.9, 2.0],
        [1.8, -2.6, 3.0],
        [0.8, 3.5, 2.5],
        [-4.0, 4.5, 2.5],
        [4.4, -4.3, 3.0],
        [1.2, -6.8, 2.0],
        [10.1, -1.1, 2.5],
        [-8.0, 8.0, 2.5],
        [2.7, -13.2, 3.0],
        [3.0, -13.3, 3.0],
        [15.5, 8.8, 2.0],
        [18.5, 13.1, 1.5],
        [-24.2, 5.7, 2.5],
        [-27.5, 8.7, 1.5],
        [20.5, 22.2, 1.2],
        [-45.1, 30.0, 2.0],
    ]
    ACTIONS = {
        0: Vector3(0.5, 0.0, 0.0),  # direita...
        1: Vector3(-0.5, 0.0, 0.0), # esquerda...
        2: Vector3(0.0, 0.5, 0.0),  # frente...
        3: Vector3(0.0, -0.5, 0.0), # trás...
        4: Vector3(0.0, 0.0, 0.5),  # subir...
        5: Vector3(0.0, 0.0, -0.5)  # descer...
    }

    def __init__(self, uav_id=1, goal=None, mode='train') -> None:
        self.uav = UAV(uav_id=uav_id)
        self.seed()

        '''
        ## observation space...
            # obstaculo mais próximo
            # vetor para o objetivo
            posição do drone (coordenadas x, y e z)
            distância até o objetivo
            vetor do objetivo
        '''

        # observation space...
        self.n_laser_vectors = 360
        self._laser_scan = self.uav.uav_info.laser_scan

        self.observation_space = spaces.Dict({
            # vetor para o objetivo
            'goal_vector': spaces.Box(low=0.0, high=np.inf, shape=(3,), dtype=np.float32),
            # n_laser_vectors lasers (vectors) para os obstáculos ordenados pela distância para o uav
            'obstacles': spaces.Box(
                low=self._laser_scan.range_min, 
                high=self._laser_scan.range_max, 
                shape=(self.n_laser_vectors, 2),
                dtype=np.float32)
        })

        # action space...
        # self.action_space = spaces.Discrete(6)
        MAX = np.finfo(np.float32).max
        no_of_actions = 3
        self.action_space = gym.spaces.Box(low=np.full(no_of_actions,-MAX,np.float32), high=np.full(no_of_actions,MAX,np.float32), dtype=np.float32)

        self.goal = goal
        self.mode = mode
        self.initial_distance_to_goal = None
        self.n_hits_on_taget = 0

        # gazebo comms...
        # self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        # mrs ros nodes...
        self.ros_waiter = ROSWaiter('uav1')

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        # self.unpause()
        # velocity = self.ACTIONS.get(action)
        velocity = Vector3(*action)
        self.uav.movements.apply_velocity(velocity)

        observation = self._get_observation()
        reward = self._calculate_reward()
        done = self._check_episode_completion()

        # print(f'reward: {reward}, observation: {observation}, action: {action}')
        # print(f'reward: {reward}')
        # print(action)
        return observation, reward, done, {}

    def reset(self):
        # rospy.loginfo('[FSPPEnv.reset]: antes _reset_mrs_nodes')
        self._reset_mrs_nodes()
        self.uav.movements.takeoff()
        # rospy.loginfo('[FSPPEnv.reset]: depois _reset_mrs_nodes')

        if self.mode == 'train':
            # gera um novo goal a cada self.N_HITS_RESET_GOAL (acerto) no target...
            if self.n_hits_on_taget % self.N_HITS_RESET_GOAL == 0:
                self.goal = self._generate_random_goal()

        self.initial_distance_to_goal = Geometry.euclidean_distance(
            self.uav.uav_info.get_uav_position(tolist=True), self.goal)
        
        rospy.loginfo('[FSPPEnv.reset]: env resetado')
        print(f'GOAL: {self.goal}, {self.n_hits_on_taget}')

        return self._get_observation()
    
    def _goal_vector(self):
        uav_position = self.uav.uav_info.get_uav_position(tolist=True)
        vec =  [
            self.goal[0] - uav_position[0],
            self.goal[1] - uav_position[1],
            self.goal[2] - uav_position[2],
        ]
        return vec

    def _distance_to_goal(self):
        uav_position = self.uav.uav_info.get_uav_position(tolist=True)
        return Geometry.euclidean_distance(uav_position, self.goal)

    def _calculate_reward(self):
        laser_scan = self.uav.uav_info.get_laser_scan()
        distance_to_goal = self._distance_to_goal()

        # goal_distance rate
        goal_distance_reward = -0.1 * distance_to_goal
        
        # obstacle rate
        if np.min(laser_scan.ranges) < 0.5: # [m] próximo de colisão !!!
            obstacle_reward = -0.05
        else:
            obstacle_reward = 0.0
        
        reward = obstacle_reward + goal_distance_reward

        if self.uav.movements.in_target(self.goal): # chegou no alvo
            reward += 100
        elif self.uav.uav_info.get_active_tracker() == 'NullTracker': # bateu e caiu
            reward -= 100

        return reward
    
    def _get_observation(self):
        uav_position = np.array(self.uav.uav_info.get_uav_position(tolist=True)[:2]) # somente x,y
        obstacles = np.array(list(self.uav.map_environment.get_obstacles_rplidar()))

        if not obstacles.any(): # laser não colidiu com nenhum obstáculo...
            obstacles = np.array([[14, 14]])

        dists = np.linalg.norm(obstacles - uav_position, axis=1)
        closests_idx = np.argsort(dists)[:self.n_laser_vectors]

        if len(closests_idx) < self.n_laser_vectors: # completa o array com o valor max do laser... não colidiu com algum obstáculo
            closests_points = np.vstack([obstacles[closests_idx], [[14, 14]] * (self.n_laser_vectors - len(closests_idx))])
        else: 
            closests_points = obstacles[closests_idx]

        # vetor para o objetivo...
        goal_vec = self._goal_vector()

        observation = {
            'goal_vector': goal_vec,
            'obstacles': closests_points,
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
        elif self.uav.uav_info.get_uav_position().z < 0.7: # voando muito baixo
            done = True
            rospy.loginfo('[FSPPEnv._check_episode_completion]: voando muito baixo...')

        return done
    
    def _reset_mrs_nodes(self):
        self._kill_nodes()
        self._start_nodes()
    
    def _start_nodes(self):
        # obtém um cenário de forma aleatória...
        scenario = random.randint(1, self.N_SCENARIOS)
        world_file = f'{worlds_dir}/worlds/tree_scenario_{scenario}.world'

        # gazebo simulation
        self._start_mrs_node(
            f'roslaunch mrs_simulation simulation.launch gui:=false world_file:={world_file}',
            waiter=self.ros_waiter.wait_for_ros)
        
        # spawner...
        self._start_mrs_node(
            'rosservice call /mrs_drone_spawner/spawn "1 $UAV_TYPE --enable-rangefinder --enable-rplidar --pos 0 0 1 0"',
            waiter=self.ros_waiter.wait_for_simulation)
        
        # uav_general automatic_start...
        self._start_mrs_node(
            'roslaunch mrs_uav_general automatic_start.launch',
            waiter=self.ros_waiter.wait_for_simulation)

        # uav_general core...
        self._start_mrs_node('roslaunch mrs_uav_general core.launch',
            waiter=self.ros_waiter.wait_for_odometry)

        # # takeoff
        # self._start_mrs_node(
        #     """
        #     rosservice call /$UAV_NAME/mavros/cmd/arming 1;
        #     sleep 2;
        #     rosservice call /$UAV_NAME/mavros/set_mode 0 offboard
        #     """,
        #     waiter=self.ros_waiter.wait_for_control)
        
        time.sleep(8)

    def _kill_nodes(self):
        kill_path = os.path.join(filepath.parent, 'kill.sh')
        subprocess.call(
            ['sh', kill_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT)
        time.sleep(2.0)

    def _start_mrs_node(self, command, waiter):
        # rospy.loginfo(f'[FSPPEnv._start_mrs_node]: cmd: {command}')
        subprocess.Popen(
            command,
            shell=True,
            env=mrs_env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT)
        waiter()

    def render(self):
        ...
    
    def close(self):
        ...