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
from std_srvs.srv import Empty

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
    TREE_HEIGHT = 6.0 # [m]
    N_HITS_RESET_GOAL = 200 # quantidade de acertos até resetar o goal
    N_SCENARIOS = 3
    POSSIBLE_GOALS = [
        [0.0, 2.0, 1.2],
        [-1.5, 1.4, 2.0],
        [0.9, 2.9, 2.5],
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

    def __init__(self, uav_id=1, goal=None) -> None:
        self.uav = UAV(uav_id=uav_id)
        self.seed()

        self.action_space = spaces.Discrete(6)

        self.observation_space = spaces.Dict({
            'goal_distance': spaces.Box(low=0.0, high=np.inf, shape=(1,), dtype=np.float32),
            'position': spaces.Box(low=-100.0, high=100.0, shape=(3,), dtype=np.float32),
            'obstacles': spaces.Box(low=0.0, high=15.0, shape=(720,), dtype=np.float32),
        })

        self.goal = goal
        self.initial_distance_to_goal = None
        self.n_hits_on_taget = 0

        # gazebo comms...
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        # mrs ros nodes...
        self.ros_waiter = ROSWaiter('uav1')

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        # self.unpause()
        velocity = self.ACTIONS.get(action)
        uav_position = self.uav.uav_info.get_uav_position(tolist=True)
        self.uav.movements.apply_velocity(velocity)
        # self.pause()

        observation = self._get_observation()
        reward = self._calculate_reward(uav_position)
        done = self._check_episode_completion()
        info = {}

        # print(f'observation: {observation}, reward: {reward}, done: {done}, info: {info}, action: {action}')
        # print(f'reward: {reward}')
        # print(action)
        return observation, reward, done, info

    def reset(self):
        # rospy.loginfo('[FSPPEnv.reset]: antes _reset_mrs_nodes')
        self._reset_mrs_nodes()
        self.uav.movements.takeoff()
        # rospy.loginfo('[FSPPEnv.reset]: depois _reset_mrs_nodes')

        # gera um novo goal a cada self.N_HITS_RESET_GOAL (acerto) no target...
        if self.n_hits_on_taget % self.N_HITS_RESET_GOAL == 0:
            self.goal = self._generate_random_goal()

        self.initial_distance_to_goal = Geometry.euclidean_distance(
            self.uav.uav_info.get_uav_position(tolist=True), self.goal)
        
        rospy.loginfo('[FSPPEnv.reset]: env resetado')

        return self._get_observation()

    def _distance_to_goal(self):
        uav_position = self.uav.uav_info.get_uav_position(tolist=True)
        return Geometry.euclidean_distance(uav_position, self.goal)

    def _calculate_reward(self, prev_uav_position):
        reward = 0
        distance_to_goal = self._distance_to_goal()
        
        if self.uav.uav_info.get_active_tracker() == 'NullTracker': # bateu e caiu
            return -1
        elif self.uav.movements.in_target(self.goal): # chegou no alvo
            return 10.0
        else:
            prev_distance_to_goal = Geometry.euclidean_distance(prev_uav_position, self.goal)
            if distance_to_goal > prev_distance_to_goal: # se distanciou do goal
                reward = -0.05
            else:
                reward = (prev_distance_to_goal - distance_to_goal) * 10.0 # mais pontos cada vez que se aproxima do goal...
            if self.uav.uav_info.get_uav_position().z > self.TREE_HEIGHT: # se estiver voando sob as árvores
                reward -= 0.05

        reward -= 0.01 # desconta a cada step...

        return reward
    
    def _get_observation(self):
        laser_scan = self.uav.uav_info.get_laser_scan()
        uav_position = self.uav.uav_info.get_uav_position(tolist=True)
        goal_distance = Geometry.euclidean_distance(uav_position, self.goal)

        # trata obstaculos..
        obstacles = np.array(laser_scan.ranges)
        obstacles = np.where(np.isinf(obstacles), laser_scan.range_max, obstacles)

        observation = {
            'goal_distance': goal_distance,
            'position': uav_position,
            'obstacles': obstacles,
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

        # takeoff
        self._start_mrs_node(
            """
            rosservice call /$UAV_NAME/mavros/cmd/arming 1;
            sleep 2;
            rosservice call /$UAV_NAME/mavros/set_mode 0 offboard
            """,
            waiter=self.ros_waiter.wait_for_control)
        
        time.sleep(10)

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