import os
import subprocess
import rospy
import numpy as np
from uav.uav import UAV
from geometry.geometry import Geometry
from geometry_msgs.msg import Vector3
import gym
from gym import spaces

os.environ['UAV_NAME'] = 'uav1'
os.environ['RUN_TYPE'] = 'simulation'
os.environ['UAV_TYPE'] = 'f450'
os.environ['WORLD_NAME'] = 'simulation_local'
os.environ['SENSORS'] = 'garmin_down'
os.environ['ODOMETRY_TYPE'] = 'gps'
os.environ['PX4_SIM_SPEED_FACTOR'] = '1'

class FSPPEnv(gym.Env):
    def __init__(self, uav_id=1) -> None:        
        
        self.uav = UAV(uav_id=uav_id)

        self.action_space = spaces.Discrete(6)

        self.observation_space = spaces.Dict({
            'goal': spaces.Box(low=-60.0, high=60.0, shape=(3,), dtype=np.float32),
            'position': spaces.Box(low=-100.0, high=100.0, shape=(3,), dtype=np.float32),
            'obstacles': spaces.Box(low=0, high=15.0, shape=(720,), dtype=np.float32),
        })

        self.goal = None
        self.initial_distance_to_goal = None

    def step(self, action):
        if action == 0: # direita...
            velocity = Vector3(0.5, 0.0, 0.0)
        elif action == 1: # esquerda...
            velocity = Vector3(-0.5, 0.0, 0.0)
        elif action == 2: # frente...
            velocity = Vector3(0.0, 0.5, 0.0)
        elif action == 3: # trás...
            velocity = Vector3(0.0, -0.5, 0.0)
        elif action == 4: # subir
            velocity = Vector3(0.0, 0.0, 0.5)
        elif action == 5: # descer...
            velocity = Vector3(0.0, 0.0, -0.5)

        uav_position = self.uav.uav_info.get_uav_position()
        self.uav.movements.apply_velocity(velocity)

        observation = self._get_observation()
        reward = self._calculate_reward(uav_position)
        done = self._check_episode_completion()
        info = {}

        return observation, reward, done, info

    def reset(self):
        self._kill_nodes()
        self._start_nodes()

        while self.uav.uav_info.get_active_tracker() == 'NullTracker': # aguarda decolar novamente...
            rospy.sleep(0.1)

        self.uav.movements.goto([0.0, 0.0, 2.0])
        self.goal = self._generate_random_goal()
        self.initial_distance_to_goal = Geometry.euclidean_distance(
            [0.0, 0.0, 2.0], self.goal)
        return self._get_observation()

    def _distance_to_goal(self):
        uav_position = self.uav.uav_info.get_uav_position()
        return Geometry.euclidean_distance(
            [uav_position.x, uav_position.y, uav_position.z], self.goal)

    def _calculate_reward(self, prev_uav_position):
        reward = 0
        distance_to_goal = self._distance_to_goal()
        
        if self.uav.uav_info.get_active_tracker() == 'NullTracker': # bateu e caiu
            reward = -50.0
        elif self.uav.movements.in_target(self.goal): # chegou no alvo
            reward = 100.0
        else:
            prev_distance_to_goal = Geometry.euclidean_distance(
                [prev_uav_position.x, prev_uav_position.y, prev_uav_position.z], self.goal)
            if distance_to_goal > prev_distance_to_goal: # se distanciou do goal
                reward = -10.0
            else:
                reward = 10.0 - (distance_to_goal * 0.5)
        return reward
    def _get_observation(self):
        # obstacles = self.uav.map_environment.get_obstacles_rplidar()
        laser_scan = self.uav.uav_info.get_laser_scan()
        uav_position = self.uav.uav_info.get_uav_position()
        observation = {
            'goal': self.goal,
            'position': (uav_position.x, uav_position.y, uav_position.z),
            'obstacles': laser_scan.ranges,
        }
        return observation

    def _generate_random_goal(self):
        x_values = np.arange(-10.0, 10.0, 0.5)
        y_values = np.arange(-10.0, 10.0, 0.5)
        z_values = np.arange(1.5, 3.0, 0.5)
        x = np.random.choice(x_values)
        y = np.random.choice(y_values)
        z = np.random.choice(z_values)
        goal = (x, y, z)
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
            rospy.loginfo('[FSPPEnv._check_episode_completion]: chegou no destino')
        elif distance_to_goal > self.initial_distance_to_goal + 5.0: # se distanciou muito do goal
            done = True
            rospy.loginfo('[FSPPEnv._check_episode_completion]: se distanciou muito do goal')

        return done
    
    def _start_nodes(self):
        subprocess.Popen(
            'roslaunch mrs_simulation simulation.launch gui:=true', 
            shell=True,
            
            )
        rospy.sleep(5.0)
        subprocess.Popen(
            'rosservice call /mrs_drone_spawner/spawn "1 $UAV_TYPE --enable-rangefinder --enable-rplidar --pos 0 0 1 0"', 
            shell=True,
            
            )
        subprocess.Popen(
            'roslaunch mrs_uav_general automatic_start.launch', 
            shell=True,
            
            )
        rospy.sleep(10.0)
        subprocess.Popen(
            'roslaunch mrs_uav_general core.launch', 
            shell=True,
            
            )
        rospy.sleep(5.0)
        subprocess.Popen(
            'rosservice call /$UAV_NAME/mavros/cmd/arming 1; sleep 2; rosservice call /$UAV_NAME/mavros/set_mode 0 offboard', 
            shell=True,
            
            )
        rospy.sleep(5.0)

    def _kill_nodes(self):
        subprocess.call(
            ['sh', './kill.sh'],
            
            )
        rospy.sleep(2.0)

    def render(self):
        ...
    
    def close(self):
        ...