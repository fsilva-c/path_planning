import rospy
from classic_path_planner.astar import AStar
from time import perf_counter
from uav.uav import UAV
from geometry.grid_map import GridMap

class GlobalPathPlannerState:
    PLANNING =  1
    OBSTACLE_FOUND = 2
    MOVING = 3
    GOAL_REACHED = 4

class GlobalPathPlanner:
    def __init__(
            self,
            goal,
            uav_id=1,
            grid_size=400,
            resolution=0.5,
            max_obstacle_dist=0.7,
        ) -> None:
        self.goal = goal
        self.uav = UAV(uav_id)
        self.grid_size = grid_size
        self.resolution = resolution
        self.max_obstacle_dist = max_obstacle_dist
        self.grid_map = None
        self.current_path = None
        self.current_state = GlobalPathPlannerState.PLANNING
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

        # para testes...
        self.uav.movements.switch_controller('Se3Controller')
        # self.uav.movements.switch_controller('MpcController')
        self.uav.movements.set_velocity('fast')

        while self.current_state != GlobalPathPlannerState.GOAL_REACHED:
            self.update()
            rospy.sleep(0.1)

    def update(self):
        if self.current_state == GlobalPathPlannerState.PLANNING:
            rospy.loginfo(f'[PathPlanner]: planejando o caminho...')
            self.path_plan()
            self.current_state = GlobalPathPlannerState.MOVING

        elif self.current_state == GlobalPathPlannerState.MOVING:
            if self.uav.movements.in_target([self.goal[0], self.goal[1]]):
                rospy.loginfo(f'[PathPlanner]: chegou no alvo...')
                self.current_state = GlobalPathPlannerState.GOAL_REACHED

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
        self.uav.movements.goto_trajectory(path)
