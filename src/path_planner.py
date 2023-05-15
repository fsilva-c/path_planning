import rospy
import geometry
from astar import AStar
from time import perf_counter
from uav import UAV
from grid_map import GridMap

class State:
    PLANNING =  1
    OBSTACLE_FOUND = 2
    MOVING = 3
    GOAL_REACHED = 4

class PathPlanner:
    def __init__(
            self,
            goal,
            uav_id=1,
            grid_size=200,
            resolution=1.0,
            max_obstacle_dist=0.7,
        ) -> None:
        self.goal = goal
        self.uav = UAV(uav_id)
        self.grid_size = grid_size
        self.resolution = resolution
        self.max_obstacle_dist = max_obstacle_dist
        self.grid_map = None
        self.current_path = None
        self.current_state = State.PLANNING
        self.attempts_obstacle_avoid = 0

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

        self.uav.movements.set_velocity('slow')

        while self.current_state != State.GOAL_REACHED:
            self.update()
            rospy.sleep(0.1)

    def update(self):
        if self.current_state == State.PLANNING:
            rospy.loginfo(f'[PathPlanner]: planejando o caminho...')
            self.path_plan()
            self.current_state = State.MOVING

        elif self.current_state == State.MOVING:
            if self.uav.movements.in_target([self.goal[0], self.goal[1]]):
                rospy.loginfo(f'[PathPlanner]: chegou no alvo...')
                self.current_state = State.GOAL_REACHED
            elif (
                self.obstacle_in_current_path() and
                self.uav.map_environment.distance_to_closest_obstacle(max_range=5.0) < self.max_obstacle_dist * 2
            ):
                rospy.loginfo(f'[PathPlanner]: obstáculos no caminho...')
                self.uav.movements.hover()
                self.path_plan()
                self.current_state = State.OBSTACLE_FOUND
        
        elif self.current_state == State.OBSTACLE_FOUND:
            if (
                not self.obstacle_in_current_path() and
                self.uav.map_environment.distance_to_closest_obstacle(max_range=5.0) > self.max_obstacle_dist * 2 + 0.5
            ):
                rospy.loginfo(f'[PathPlanner]: sem obstáculos no caminho...')
                self.current_state = State.MOVING
            elif self.attempts_obstacle_avoid >= 5:
                rospy.loginfo(f'[PathPlanner]: alcançou o máximo de tentativas...')
                self.current_state = State.PLANNING
            else:
                rospy.loginfo(f'[PathPlanner]: aguardando sair da zona de perigo...')
                self.attempts_obstacle_avoid += 1
                rospy.sleep(1.0)

    def obstacle_in_current_path(self):
        obstacles = self.uav.map_environment.get_obstacles()
        for point in self.current_path:
            for obstacle in obstacles:
                distance_to_path = geometry.euclidean_distance(obstacle, point[:2])
                if (distance_to_path < self.max_obstacle_dist):
                    return True
        return False

    def path_plan(self):
        rospy.loginfo('[PathPlanner]: Path Planning...')
        time_start = perf_counter()
        obstacles = self.uav.map_environment.get_obstacles()
        
        # adiciona os obstáculos no grid...
        for obstacle in obstacles:
            x, y = round(obstacle[0] * 2) / 2, round(obstacle[1] * 2) / 2
            self.grid_map.set_value_from_xy_pos(x_pos=x, y_pos=y, val=1.0)
        
        # expande os obstáculos..
        self.grid_map.expand_grid()
        
        # find path...
        uav_position = self.uav.uav_info.get_uav_position()
        astar = AStar(self.grid_map)
        path = astar.find_path(
            start=(round(uav_position.x * 2) / 2, round(uav_position.y * 2) / 2),
            goal=self.goal
        )
        rospy.loginfo(f'[PathPlanner]: O planejamento levou {round(perf_counter() - time_start, 5)}s para ser concluído...')
        # print(path)
        if path:
            self.current_path = path
            self.uav.movements.goto_trajectory(path)
        else:
            rospy.loginfo(f'[PathPlanner]: O planejamento falhou...')
    