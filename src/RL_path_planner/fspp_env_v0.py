import gym
import numpy as np
from gym import spaces
from uav.uav import UAV
from geometry.geometry import Geometry
from geometry_msgs.msg import Vector3

class FSPPEnv(gym.Env):
    def __init__(self, uav_id=1) -> None:
        self.uav = UAV(uav_id=uav_id)

        self.action_space = spaces.Discrete(6)
        self.observation_space = spaces.Box(low=-100.0, high=100.0, shape=(3,), dtype=np.float32)
        # self.observation_space = spaces.Dict({
        #     'position': spaces.Box(low=-100.0, high=100.0, shape=(3,), dtype=np.float32),
        #     'obstacles': spaces.Box(low=-15.0, high=15.0, shape=(3,), dtype=np.float32),
        #     'garmin': spaces.Box(low=0.0, high=30.0, shape=(1,), dtype=np.float32)
        # })

        self.goal = None
    
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

        # zera para o drone n descer de mais
        if action == 5 and self.uav.uav_info.get_garmin_range() < 1.0:
            velocity = Vector3(0.0, 0.0, 0.0)

        prev_uav_position = self.uav.uav_info.get_uav_position()
        self.uav.movements.apply_velocity(velocity)

        observation = self._get_observation()
        reward = self._calculate_reward(prev_uav_position)
        done = self._check_episode_completion()
        info = {}

        return observation, reward, done, info

    def reset(self):
        self.goal = self._generate_random_goal()
        return self._get_observation()

    def _calculate_reward(self, prev_uav_position):
        uav_position = self.uav.uav_info.get_uav_position()
        distance_to_goal = Geometry.euclidean_distance(
            [uav_position.x, uav_position.y, uav_position.z], self.goal)
        if self.uav.movements.in_target(self.goal):
            reward = 100.0
        else:
            prev_distance_to_goal = Geometry.euclidean_distance(
                [prev_uav_position.x, prev_uav_position.y, prev_uav_position.z], self.goal)
            if distance_to_goal < prev_distance_to_goal:
                reward = 1.0
            else:
                reward = -2.0
        return reward

    def _get_observation(self):
        uav_position = self.uav.uav_info.get_uav_position()
        x = uav_position.x
        y = uav_position.y
        z = uav_position.z
        observation = (x, y, z)
        # obstacles = self.uav.map_environment.get_obstacles_rplidar()
        # garmin = self.uav.uav_info.get_garmin_range()
        # observation = {
        #     'position': (x, y, z),
        #     'obstacles': obstacles,
        #     'garmin': garmin
        # }
        return observation

    def _generate_random_goal(self):
        x_values = np.arange(-6.0, 6.0, 0.5)
        y_values = np.arange(-6.0, 6.0, 0.5)
        z_values = np.arange(1.5, 3.0, 0.5)
        x = np.random.choice(x_values)
        y = np.random.choice(y_values)
        z = np.random.choice(z_values)
        goal = (x, y, z)
        print(f'GOAL: {goal}')
        return goal

    def _check_episode_completion(self): # verificando se o episódio terminou...
        if self.uav.movements.in_target(self.goal): # chegou no destino
            done = True
        elif self.uav.uav_info.get_active_tracker() == 'NullTracker': # bateu e caiu
            done = True
        else:
            done = False
        return done

    def render(self):
        ...
    
    def close(self):
        ...