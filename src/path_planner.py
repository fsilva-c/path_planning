import rospy
import numpy as np
from uav import UAV
from astar import Astar

class State:
    PLANNING =  1
    OBSTACLE_FOUND = 2
    MOVING = 3
    GOAL_REACHED = 4

class PathPlanner:
    def __init__(self, goal, uav_id=1, resolution=1.0, grid_size=150, max_obstacle_dist=0.45) -> None:
        self.uav = UAV(uav_id)

        self.goal = goal
        self.resolution = resolution # tamanho de cada célula
        self.grid_size = grid_size
        self.max_obstacle_dist = max_obstacle_dist
        self.max_attempts_obstacle_avoid = 0

        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)

        self.current_path = None
        self.current_state = State.PLANNING

    def run(self):
        uav_position = self.uav.uav_info.get_uav_position()

        rospy.loginfo('[PathPlanner]: Start Path Planning...')
        rospy.loginfo(
            f'[PathPlanner]: Start {uav_position.x, uav_position.y}; Goal: {self.goal[0], self.goal[1]}...'
        )

        while self.current_state != State.GOAL_REACHED:
            # rospy.loginfo(f'[PathPlanner]: Current State: {self.current_state}...')
            self.update()
            rospy.sleep(0.1)

    def update(self):
        if self.current_state == State.PLANNING:
            rospy.loginfo(f'[PathPlanner]: planejando o caminho...')
            self.path_plan()
            self.current_state = State.MOVING
            self.max_attempts_obstacle_avoid = 0

        elif self.current_state == State.MOVING:
            if self.uav.movements.in_target([self.goal[0], self.goal[1]]):
                rospy.loginfo(f'[PathPlanner]: chegou no alvo...')
                self.current_state = State.GOAL_REACHED
            elif self.obstacle_in_current_path():
                rospy.loginfo(f'[PathPlanner]: obstáculos no caminho...')
                self.uav.movements.hover()
                self.path_plan()
                self.current_state = State.OBSTACLE_FOUND
        
        elif self.current_state == State.OBSTACLE_FOUND:
            if not self.obstacle_in_current_path():
                rospy.loginfo(f'[PathPlanner]: sem obstáculos no caminho...')
                self.current_state = State.MOVING
            elif self.max_attempts_obstacle_avoid >= 5:
                rospy.loginfo(f'[PathPlanner]: alcançou o máximo de tentativas...')
                self.current_state = State.PLANNING
            else:
                rospy.loginfo(f'[PathPlanner]: aguardando sair da zona de perigo...')
                self.max_attempts_obstacle_avoid += 1
                rospy.sleep(0.5)
                
    def path_plan(self):
        rospy.loginfo('[PathPlanner]: Path Planning...')

        obstacles = self.get_obstacles()
        uav_position = self.uav.uav_info.get_uav_position()
        rel = self.grid_size // 2
        drone_pos = (int(round(uav_position.x / self.resolution + rel)), int(round(uav_position.y / self.resolution + rel)))
        goal_pos = (int(round(self.goal[0] + rel)), int(round(self.goal[1] + rel)))

        # adiciona os obstáculos no grid...
        for obstacle in obstacles:
            pos = np.array([uav_position.x, uav_position.y])
            pos = np.round(pos, decimals=0)

            x = int(round(obstacle[0] + pos[0] / self.resolution + self.grid_size / 2))
            y = int(round(obstacle[1] + pos[1] / self.resolution + self.grid_size / 2))

            self.grid[x, y] = 1

        path = Astar(self.grid, drone_pos, goal_pos).a_star()
        converted_path = [
            ((p[0] - rel) * self.resolution, (p[1] - rel) * self.resolution, uav_position.z) for p in path
        ]
        self.current_path = converted_path
        self.uav.movements.goto_trajectory(converted_path)

    def obstacle_in_current_path(self):
        obstacles = self.get_obstacles(around=False)
        for point in self.current_path:
            for obstacle in obstacles:
                distance = np.linalg.norm(point[:1] - obstacle)
                if distance < self.max_obstacle_dist:
                    return True
        return False
    
    def get_obstacles(self, around=True):
        # agrupando os obstáculos...
        point_cloud = self.uav.hector.get_point_cloud()
        obstacles = np.array([[point.x, point.y] for point in point_cloud.points])
        if around:
            rounded_data = np.around(obstacles)
            obstacles = np.unique(rounded_data, axis=0)
        return obstacles
    