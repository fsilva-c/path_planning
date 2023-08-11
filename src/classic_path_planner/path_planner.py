import rospy
from classic_path_planner.astar import AStar
from time import perf_counter
from uav.uav import UAV
from geometry.geometry import Geometry
from geometry.discrete_grid import DiscreteGrid

class StatePlanner:
    PLANNING = 1
    MOVING = 2
    OBSTACLE_FOUND = 3
    GOAL_REACHED = 4

class PathPlanner:
    def __init__(
            self,
            goal,
            uav_id=1,
            resolution=0.5,
            threshold=1.0
        ) -> None:
        self.goal = goal
        self.uav = UAV(uav_id)
        self.threshold = threshold
        self.dg = DiscreteGrid(resolution)
        self.current_state = StatePlanner.PLANNING
        self.current_path = []
        self.start = []

    def run(self) -> None:
        uav_position = self.uav.uav_info.get_uav_position()

        rospy.loginfo('[PathPlanner]: Start Path Planning...')
        rospy.loginfo(
            f'[PathPlanner]: Start {uav_position.x, uav_position.y, uav_position.z}; Goal: {self.goal}...'  # noqa: E501
        )

        while self.current_state != StatePlanner.GOAL_REACHED:
            if self.current_state == StatePlanner.PLANNING:
                self.path_plan()
                self.current_state = StatePlanner.MOVING
            
            elif self.current_state == StatePlanner.MOVING:
                # if self.distance_to_start() > 8.0:
                if self.has_obstacles_in_current_path():
                    self.current_state = StatePlanner.OBSTACLE_FOUND
                if self.uav.movements.in_target(list(self.goal)):
                    self.current_state = StatePlanner.GOAL_REACHED

            elif self.current_state == StatePlanner.OBSTACLE_FOUND:
                self.current_state = StatePlanner.PLANNING
            
            rospy.sleep(0.01)

        rospy.loginfo('[PathPlanner]: Finalizado Path Planning...')

    def obstacles(self) -> list:
        uav_position = self.uav.uav_info.get_uav_position()

        obstacles = {(uav_position.x + x, uav_position.y + y, uav_position.z)
                 for x, y in self.uav.map_environment.get_obstacles_rplidar()}
        
        obstacles.update((uav_position.x + x, uav_position.y + y, uav_position.z + z)
                     for x, y, z in self.uav.map_environment.get_obstacles_realsense())
        
        return list(obstacles)

    def has_obstacles_in_current_path(self) -> bool:
        obstacles = self.obstacles()
        min_distance = 0.4
        return any(
            Geometry.euclidean_distance(point, obstacle) < min_distance
            for point in self.current_path
            for obstacle in obstacles
        )

    def distance_to_start(self) -> float:
        uav_position = self.uav.uav_info.get_uav_position()
        return Geometry.euclidean_distance(self.start, [uav_position.x, uav_position.y, uav_position.z])

    def path_plan(self) -> None:
        rospy.loginfo('[PathPlanner]: Path Planning...')

        uav_position = self.uav.uav_info.get_uav_position()
        self.start = (uav_position.x, uav_position.y, uav_position.z)

        time_start = perf_counter()
        rospy.loginfo('[PathPlanner]: Encontrando o caminho...')
        path = AStar(
            threshold=self.threshold,
            dg=self.dg,
            obstacles=self.obstacles()
        ).find_path(start=self.start, goal=self.goal)

        path.append(self.goal)
        self.current_path = path
        # path = Geometry.apply_cubic_spline(path)
        # path = Geometry.remove_collinear_points(path)
        rospy.loginfo('[PathPlanner]: Caminho encontrado...')
        rospy.loginfo(f'[PathPlanner]: O planejamento levou {round(perf_counter() - time_start, 5)}s para ser concluído...')
        self.uav.movements.goto_trajectory(path)
