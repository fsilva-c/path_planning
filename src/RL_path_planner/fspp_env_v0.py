import rospy
import os
import numpy as np
from uav.uav import UAV
from std_srvs.srv import Empty
from geometry.geometry import Geometry
from geometry_msgs.msg import Vector3
import gym
from gym import spaces

class FSPPEnv(gym.Env):
    def __init__(self, uav_id=1) -> None:
        self.uav = UAV(uav_id=uav_id)

        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)

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
        # try:
        #     self.reset_proxy()
        # except rospy.ServiceException as e:
        #     rospy.logerr(f'[FSPPEnv.reset]: fail reset_proxy(): {e}')
        # self.uav.movements.takeoff()
   
   
        # import roslaunch

        # roslaunch.core.Node()
        
        # nodes = os.popen("rosnode list").readlines()
        # for node in nodes:
        #     node = node.replace('\n', '')
        #     if '/uav1/' in node:
        #         os.system(f'rosnode kill {node}')

        # rospy.sleep(5)

        # uav_position = self.uav.uav_info.get_uav_position()

        '''
        # rosnode list...
        /gazebo
        /mrs_drone_spawner
        /rosout
        /uav1/automatic_start
        /uav1/constraint_manager
        /uav1/control_manager
        /uav1/gain_manager
        /uav1/mavros
        /uav1/mavros_diagnostics
        /uav1/mrs_uav_status
        /uav1/mrs_uav_status_acquisition
        /uav1/odometry
        /uav1/rtk_republisher
        /uav1/trajectory_generation
        /uav1/uav1_nodelet_manager
        /uav1/uav_manager

        # yml...
        -gazebo: waitForRos; roslaunch mrs_simulation simulation.launch gui:=false 
        -spawn: waitForSimulation; rosservice call /mrs_drone_spawner/spawn "1 $UAV_TYPE --enable-rangefinder --enable-rplidar --pos_file `pwd`/pos.yaml"
        -control: waitForOdometry; roslaunch mrs_uav_general core.launch
        -takeoff: waitForSimulation; roslaunch mrs_uav_general automatic_start.launch
        -takeoff: 'waitForControl;
          rosservice call /$UAV_NAME/mavros/cmd/arming 1;
          sleep 2;
          rosservice call /$UAV_NAME/mavros/set_mode 0 offboard'
        '''

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

    def render(self):
        ...
    
    def close(self):
        ...