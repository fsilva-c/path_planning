import rospy
from classic_path_planner.astar import AStar
from time import perf_counter
from uav.uav import UAV
from geometry.grid_map import GridMap
from geometry.geometry import Geometry

from classic_path_planner.local_planner import LocalPathPlanner

class PathPlannerState:
    PLANNING =  1
    MOVING = 2
    GOAL_REACHED = 3

class GlobalPathPlanner:
    def __init__(
            self,
            goal,
            uav_id=1,
            grid_size=200,
            resolution=1.0,
            max_obstacle_dist=0.7,
        ) -> None:
        self.start = None
        self.goal = goal
        self.uav = UAV(uav_id)
        self.grid_size = grid_size
        self.resolution = resolution
        self.max_obstacle_dist = max_obstacle_dist
        self.grid_map = None
        self.current_path = None
        self.current_state = PathPlannerState.PLANNING
        self.attempts_obstacle_avoid = 0
        self.lpp = None

        self.on_init()

    def on_init(self):
        uav_position = self.uav.uav_info.get_uav_position()

        self.grid_map = GridMap(
            width=self.grid_size,
            height=self.grid_size,
            resolution=self.resolution,
            center_x=uav_position.x,
            center_y=uav_position.y
        )

        self.lpp = LocalPathPlanner(
            uav_id=self.uav.id
        )

    def run(self):
        uav_position = self.uav.uav_info.get_uav_position()

        rospy.loginfo('[PathPlanner]: Start Path Planning...')
        rospy.loginfo(
            f'[PathPlanner]: Start {uav_position.x, uav_position.y}; Goal: {self.goal[0], self.goal[1]}...'  # noqa: E501
        )

        # para testes...
        self.uav.movements.switch_controller('MpcController') # Se3Controller
        self.uav.movements.set_velocity('slow')

        while self.current_state != PathPlannerState.GOAL_REACHED:
            self.update()
            rospy.sleep(0.01)

    def update(self):
        if self.current_state == PathPlannerState.PLANNING:
            rospy.loginfo(f'[PathPlanner]: planejando o caminho...')
            self.path_plan()
            self.current_state = PathPlannerState.MOVING

        elif self.current_state == PathPlannerState.MOVING:
            if self.uav.movements.in_target([self.goal[0], self.goal[1]]):
                rospy.loginfo(f'[PathPlanner]: chegou no alvo...')
                self.current_state = PathPlannerState.GOAL_REACHED
            elif self.distance_to_start() >= 10.0:
                rospy.loginfo(f'[PathPlanner]: distância máxima andada alcançada. replanejando a rota...')
                self.current_state = PathPlannerState.PLANNING

    def distance_to_start(self):
        uav_position = self.uav.uav_info.get_uav_position()
        return Geometry.norm(self.start, [uav_position.x, uav_position.y])

    def path_plan(self):
        rospy.loginfo('[PathPlanner]: Path Planning...')
        time_start = perf_counter()
        uav_position = self.uav.uav_info.get_uav_position()
        self.start = (round(uav_position.x * 2) / 2, round(uav_position.y * 2) / 2)
        obstacles = self.uav.map_environment.get_obstacles()

        # adiciona os obstáculos no grid...
        for obstacle in obstacles:
            x, y = round(obstacle[0] * 2) / 2, round(obstacle[1] * 2) / 2
            self.grid_map.set_value_from_xy_pos(x_pos=x, y_pos=y, val=1.0)
        
        # expande os obstáculos..
        self.grid_map.expand_grid()
        # self.grid_map.expand_grid()
        
        # find path...
        astar = AStar(self.grid_map)
        path = astar.find_path(
            start=self.start,
            goal=self.goal
        )
        self.current_path = path
        rospy.loginfo(f'[PathPlanner]: O planejamento levou {round(perf_counter() - time_start, 5)}s para ser concluído...')
        self.uav.movements.goto_trajectory(path)
