import rospy
from classic_path_planner.astar import AStar
from time import perf_counter
from uav.uav import UAV
from geometry.grid_map import GridMap
from geometry.geometry import Geometry

class PathPlanner:
    def __init__(
            self,
            goal,
            uav_id=1,
            grid_size=200,
            resolution=0.5,
        ) -> None:
        self.start = None
        self.goal = goal
        self.uav = UAV(uav_id)
        self.grid_size = grid_size
        self.resolution = resolution
        self.grid_map = None

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

    def run(self):
        uav_position = self.uav.uav_info.get_uav_position()

        rospy.loginfo('[PathPlanner]: Start Path Planning...')
        rospy.loginfo(
            f'[PathPlanner]: Start {uav_position.x, uav_position.y}; Goal: {self.goal[0], self.goal[1]}...'  # noqa: E501
        )

        self.path_plan()

        while not self.uav.movements.in_target([self.goal[0], self.goal[1]]):
            if self.distance_to_start() >= 5.0:
                self.path_plan()
            # self.uav.movements.set_heading(0.0)
            # print(f'Distance: {self.uav.map_environment.distance_to_closest_obstacle()}')
            rospy.sleep(0.1)

    def distance_to_start(self):
        uav_position = self.uav.uav_info.get_uav_position()
        return Geometry.norm(self.start, [uav_position.x, uav_position.y])

    def path_plan(self):
        rospy.loginfo('[PathPlanner]: Path Planning...')
        time_start = perf_counter()

        uav_position = self.uav.uav_info.get_uav_position()

        grid_map = GridMap(
            width=self.grid_size,
            height=self.grid_size,
            resolution=self.resolution,
            center_x=uav_position.x,
            center_y=uav_position.y
        )

        self.start = (uav_position.x, uav_position.y)
        obstacles = self.uav.map_environment.get_obstacles()

        # adiciona os obstáculos no grid...
        for obstacle in obstacles:
            x, y = obstacle
            grid_map.set_value_from_xy_pos(x_pos=x, y_pos=y, val=1.0)
        
        # expande os obstáculos..
        grid_map.expand_grid()
        
        # find path...
        path = AStar(grid_map).find_path(
            start=self.start,
            goal=self.goal
        )

        path.append(self.goal)
        rospy.loginfo(f'[PathPlanner]: O planejamento levou {round(perf_counter() - time_start, 5)}s para ser concluído...')        
        self.uav.movements.goto_trajectory(path)
