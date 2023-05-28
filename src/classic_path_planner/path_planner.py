import rospy
import math
from classic_path_planner.astar import AStar
from time import perf_counter
from uav.uav import UAV
from geometry.geometry import Geometry
from geometry.discrete_grid import DiscreteGrid2D

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
        self.resolution = resolution
        self.dg = DiscreteGrid2D(0.5)
        self.current_path = []
        self.current_state = StatePlanner.PLANNING
        self.start = None

    def run(self):
        uav_position = self.uav.uav_info.get_uav_position()

        rospy.loginfo('[PathPlanner]: Start Path Planning...')
        rospy.loginfo(
            f'[PathPlanner]: Start {uav_position.x, uav_position.y}; Goal: {self.goal[0], self.goal[1]}...'  # noqa: E501
        )

        while self.current_state != StatePlanner.GOAL_REACHED:
            if self.current_state == StatePlanner.PLANNING:
                self.path_plan()
                self.current_state = StatePlanner.MOVING
            
            elif self.current_state == StatePlanner.MOVING:
                # if self.has_obstacles_in_current_path():
                if self.distance_to_start() > 8.0:
                    self.current_state = StatePlanner.OBSTACLE_FOUND
                if self.uav.movements.in_target(list(self.goal)):
                    self.current_state = StatePlanner.GOAL_REACHED

            elif self.current_state == StatePlanner.OBSTACLE_FOUND:
                self.current_state = StatePlanner.PLANNING
            
            rospy.sleep(0.01)

        rospy.loginfo('[PathPlanner]: Finalizado Path Planning...')
        '''
        while not self.uav.movements.in_target([self.goal[0], self.goal[1]]):
            if self.distance_to_start() >= 2.0:
                self.path_plan()
            # print(f'Distance: {self.uav.map_environment.distance_to_closest_obstacle()}')
            rospy.sleep(0.01)
        '''

    def obstacles(self):
        uav_position = self.uav.uav_info.get_uav_position()
        obstacles = set()
        for p in self.uav.map_environment.get_obstacles():
            x, y = p
            point = uav_position.x + x, uav_position.y + y
            obstacles.add(point)
        return obstacles

    def has_obstacles_in_current_path(self):
        for point in self.current_path:
            # for obstacle in self.uav.map_environment.get_obstacles():
            for obstacle in self.obstacles():
                if Geometry.euclidean_distance(point, obstacle) < 0.4:
                    return True
        return False

    def set_heading(self):
        # gira o drone pra frente do alvo...
        rospy.loginfo('[PathPlanner]: Set Heading...')
        uav_position = self.uav.uav_info.get_uav_position()
        dx = self.goal[0] - uav_position.x
        dy = self.goal[1] - uav_position.y
        ref_heading = math.atan2(dy, dx)
        self.uav.movements.set_heading(ref_heading)

    def distance_to_start(self):
        uav_position = self.uav.uav_info.get_uav_position()
        return Geometry.norm(self.start, [uav_position.x, uav_position.y, uav_position.z])

    def path_plan(self):
        rospy.loginfo('[PathPlanner]: Path Planning...')
        time_start = perf_counter()

        # self.set_heading()

        uav_position = self.uav.uav_info.get_uav_position()
        self.start = (uav_position.x, uav_position.y, uav_position.z)

        rospy.loginfo('[PathPlanner]: Encontrando o caminho...')
        path = AStar(
            threshold=self.threshold,
            dg=self.dg,
            obstacles=list(self.obstacles())
        ).find_path(start=self.start, goal=self.goal)

        path = self.remove_collinear_points(path)
        self.current_path = path
        path.append(self.goal)
        # print(path)
        rospy.loginfo(f'[PathPlanner]: O planejamento levou {round(perf_counter() - time_start, 5)}s para ser concluído...')
        self.uav.movements.goto_trajectory(path, fly_now=True)

    def remove_collinear_points(self, original_path):
        new_path = []
        length = len(original_path) - 2
        new_path.append(original_path[0])

        for i in range(length):
            distance13 = Geometry.euclidean_distance(original_path[i + 2], original_path[i])
            distance12 = Geometry.euclidean_distance(original_path[i + 1], original_path[i])
            distance23 = Geometry.euclidean_distance(original_path[i + 2], original_path[i + 1])
            if abs(distance13 - distance12 - distance23) < 0.001:
                continue
            else:
                new_path.append(original_path[i + 1])

        return new_path