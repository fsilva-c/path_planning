import os
import time
import subprocess
import rospy
import pathlib
import numpy as np
from uav_interface.uav import UAV
from RL_path_planner.ros_waiter import ROSWaiter
from geometry.geometry import Geometry
from geometry_msgs.msg import Vector3
from RL_path_planner.worlds import probabilistics_forest

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
    MAX_DISTANCE = 5.0 # [m] distância máxima do goal...
    N_HITS_RESET_GOAL = 200 # quantidade de acertos até resetar o goal
    MAX_EPISODE_DURATION = rospy.Duration(180.0) # [s] tempo máximo para cada episódio
    MAX_CURRICULUM_LEARNING = 50 # [m] distância máxima da dificuldade...

    def __init__(
            self, 
            uav_id=1, 
            start=None,
            goal=None,
            act_space='continuos',
            mode='train'
        ) -> None:
        self.uav = UAV(uav_id=uav_id)
        self.start = start
        self.goal = goal
        self.act_space = act_space
        self.mode = mode
        self.seed()

        '''
            ## observation space...
            # vetor para o objetivo (ponto 3D)
            # leituras do sensor RPLidar (ranges)
        '''

        # observation space...
        num_laser_readings = 720
        low = np.array([-100.0, -100.0, -100.0] + [0.0] * num_laser_readings)
        high = np.array([100.0, 100.0, 100.0] + [15.0] * num_laser_readings)
        self.observation_space = spaces.Box(np.float32(low), np.float32(high), dtype=np.float32)

        # action space...
        if self.act_space == 'continuos':
            no_of_actions = 3
            self.action_space = gym.spaces.Box(
                low=np.full(no_of_actions,-1,np.float32),
                high=np.full(no_of_actions,1,np.float32), 
                dtype=np.float32
            )
        else:
            self.discrete_actions = {
                0: Vector3(0.5, 0.0, 0.0),  # direita...
                1: Vector3(-0.5, 0.0, 0.0), # esquerda...
                2: Vector3(0.0, 0.5, 0.0),  # frente...
                3: Vector3(0.0, -0.5, 0.0), # trás...
                4: Vector3(0.0, 0.0, 0.5),  # subir...
                5: Vector3(0.0, 0.0, -0.5), # descer...
                6: Vector3(0.0, 0.0, 0.0)   # parado...
            }
            self.action_space = spaces.Discrete(7)

        self.initial_distance_to_goal = None
        self.n_hits_on_target = 0
        self.ros_waiter = ROSWaiter('uav1') # mrs ros nodes...
        self.curriculum_learning = 1
        self.episode_duration = None
        self.n_hits_on_target = 0

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        if self.act_space == 'continuos':
            velocity = Vector3(*action)
        else:
            velocity = self.discrete_actions.get(action)
        self.uav.movements.apply_velocity(velocity)

        observation = self._get_observation()
        reward = self._calculate_reward()
        done = self._check_episode_completion()
        # print(f'observation: {observation}, reward: {reward}, action: {action}')
        # print(f'reward: {reward}')
        # print(action)
        return observation, reward, done, {}

    def reset(self):
        self._reset_mrs_nodes()
        self.uav.movements.takeoff()
        self.initial_distance_to_goal = Geometry.euclidean_distance(
            self.uav.uav_info.get_uav_position(tolist=True), self.goal)
        self.episode_duration = rospy.Time.now()
        rospy.loginfo('[FSPPEnv.reset]: env resetado')
        print(f'GOAL: {self.goal}, {self.n_hits_on_target}')
        return self._get_observation()
    
    def _goal_vector(self):
        uav_position = self.uav.uav_info.get_uav_position(tolist=True)
        vec = np.array([
            self.goal[0] - uav_position[0],
            self.goal[1] - uav_position[1],
            self.goal[2] - uav_position[2],
        ])
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
            obstacle_reward = -0.1
        else:
            obstacle_reward = 0.0

        # ended episode rate
        if self.uav.movements.in_target(self.goal): # chegou no alvo
            end_reward = 100.0
        elif self.uav.uav_info.get_active_tracker() == 'NullTracker': # bateu e caiu
            end_reward = -100.0
        else:
            end_reward = 0.0

        reward = goal_distance_reward + obstacle_reward + end_reward

        return reward

    def _get_observation(self):
        laser_scan = self.uav.uav_info.get_laser_scan()
        obstacles = np.array(laser_scan.ranges)
        obstacles = np.where(np.isinf(obstacles), laser_scan.range_max, obstacles)
        goal_vec = self._goal_vector()
        return np.concatenate([goal_vec, obstacles], dtype=np.float32)
    
    def _eval_curriculum_learning(self):
        self.n_hits_on_target += 1
        if self.n_hits_on_target % self.N_HITS_RESET_GOAL == 0:
            self.curriculum_learning = min(self.curriculum_learning + 1, self.MAX_CURRICULUM_LEARNING)

    def _check_episode_completion(self): # verificando se o episódio terminou...
        done = False
        if self.uav.uav_info.get_active_tracker() == 'NullTracker': # bateu e caiu
            done = True
            rospy.loginfo('[FSPPEnv._check_episode_completion]: bateu e caiu')
        elif self.uav.movements.in_target(self.goal): # chegou no destino
            done = True
            self._eval_curriculum_learning()
            rospy.loginfo('[FSPPEnv._check_episode_completion]: chegou no destino')
        elif self._distance_to_goal() > self.initial_distance_to_goal + self.MAX_DISTANCE: # se distanciou muito do goal
            done = True
            rospy.loginfo('[FSPPEnv._check_episode_completion]: se distanciou muito do goal')
        elif self.uav.uav_info.get_uav_position().z < 0.7: # voando muito baixo
            done = True
            rospy.loginfo('[FSPPEnv._check_episode_completion]: voando muito baixo...')
        elif rospy.Time.now() - self.episode_duration >= self.MAX_EPISODE_DURATION:
            done = True
            rospy.loginfo('[FSPPEnv._check_episode_completion]: timeout...')
        return done
    
    def _reset_mrs_nodes(self):
        self._kill_nodes()
        self._start_nodes()
    
    def _start_nodes(self):
        # obtém um cenário de forma aleatória...
        coords = probabilistics_forest.apply_distribution()
        safe_points = probabilistics_forest.find_safe_points(coords, dist=self.curriculum_learning)
        if self.mode == 'train':
            self.start = safe_points[0]
            self.goal = [*safe_points[1], np.random.uniform(1, 5)]

        # gazebo simulation
        self._start_mrs_node(
            f'roslaunch mrs_simulation simulation.launch gui:=false world_file:={filepath}/worlds/probabilistics/forest.world',
            waiter=self.ros_waiter.wait_for_ros)
        
        # spawner...
        self._start_mrs_node(
            f'rosservice call /mrs_drone_spawner/spawn "1 $UAV_TYPE --enable-rangefinder --enable-rplidar --pos {self.start[0]} {self.start[1]} 1 0"',
            waiter=self.ros_waiter.wait_for_simulation)
        
        # uav_general automatic_start...
        self._start_mrs_node(
            'roslaunch mrs_uav_general automatic_start.launch',
            waiter=self.ros_waiter.wait_for_simulation)
        
        # uav_general core...
        self._start_mrs_node('roslaunch mrs_uav_general core.launch',
            waiter=self.ros_waiter.wait_for_odometry)        
        
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