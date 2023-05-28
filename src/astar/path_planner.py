import rospy
import math
import numpy as np
from geometry.geometry import Geometry
import sensor_msgs.point_cloud2 as pc2
from time import perf_counter
from uav.uav import UAV
from geometry.discrete_grid import DiscreteGrid
from astar.astar3D import AStar3D
# from astar.astar3D_GridMap3D import AStar3D
from geometry.grid_map3D import GridMap3D

class StatePlanner:
    PLANNING = 1
    MOVING = 2
    OBSTACLE_FOUND = 3
    GOAL_REACHED = 4

class PathPlanner:
    def __init__(self, goal, uav_id=1) -> None:
        self.goal = goal
        self.uav_id = uav_id
        self.dg = DiscreteGrid(resolution=1)
        self.uav = UAV(uav_id)
        self.start = None
        self.current_state = StatePlanner.PLANNING

    def run(self):
        uav_position = self.uav.uav_info.get_uav_position()
        self.uav.movements.switch_controller('MpcController')

        rospy.loginfo('[PathPlanner]: Start Path Planning...')
        rospy.loginfo(
            f'[PathPlanner]: Start {uav_position.x, uav_position.y, uav_position.z}; Goal: {self.goal}...'  # noqa: E501
        )

        while self.current_state != StatePlanner.GOAL_REACHED:
            if self.current_state == StatePlanner.PLANNING:
                self.path_plan()
                self.current_state = StatePlanner.MOVING
            
            elif self.current_state == StatePlanner.MOVING:
                if self.distance_to_start() >= 2.0:
                    self.current_state = StatePlanner.OBSTACLE_FOUND
                elif self.uav.movements.in_target(self.goal):
                    self.current_state = StatePlanner.GOAL_REACHED

            elif self.current_state == StatePlanner.OBSTACLE_FOUND:
                self.current_state = StatePlanner.PLANNING

        '''
        while not self.uav.movements.in_target(self.goal):
            if self.distance_to_start() >= 3.0:
                self.path_plan()
            rospy.sleep(0.1)
        '''

    def distance_to_start(self):
        uav_position = self.uav.uav_info.get_uav_position()
        return Geometry.euclidean_distance(self.start, [uav_position.x, uav_position.y, uav_position.z])
    
    def set_heading(self):
        # gira o drone pra frente do alvo...
        rospy.loginfo('[PathPlanner]: Set Heading...')
        uav_position = self.uav.uav_info.get_uav_position()
        dx = self.goal[0] - uav_position.x
        dy = self.goal[1] - uav_position.y
        ref_heading = math.atan2(dy, dx)
        self.uav.movements.set_heading(ref_heading)

    def path_plan(self):
        rospy.loginfo('[PathPlanner]: Path Planning...')
        time_start = perf_counter()

        uav_position = self.uav.uav_info.get_uav_position()
        self.start = (uav_position.x, uav_position.y, uav_position.z)

        self.set_heading()

        '''
        AStar3D Gridmap...
        gm3d = GridMap3D(
            resolution=1.0,
            dimension=(100, 100, 100),
            center=self.start
        )

        point_cloud_2 = self.uav.uav_info.get_point_cloud_2()
        for p in pc2.read_points(point_cloud_2, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = p
            gm3d.insert_obstacle(
                np.array([z  + uav_position.x, -x  + uav_position.y, y  + uav_position.z]))

        path = AStar3D(gm3d).find_path(start=self.start, goal=self.goal)
        '''

        rospy.loginfo('[PathPlanner]: Find Path...')
        obstacles = set()
        point_cloud_2 = self.uav.uav_info.get_point_cloud_2()
        for p in pc2.read_points(point_cloud_2, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = p
            obstacles.add((z  + uav_position.x, -x  + uav_position.y, y  + uav_position.z))

        path = AStar3D(list(obstacles)).find_path(self.start, self.goal)  
        path.append(self.goal)
        rospy.loginfo(f'[PathPlanner]: O planejamento levou {round(perf_counter() - time_start, 5)}s para ser concluído...')        
        self.uav.movements.goto_trajectory(path)
