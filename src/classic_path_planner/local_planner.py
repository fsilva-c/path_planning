'''
seria bom criar uma classe Planner pra ter os atributos relacionados
(grid, resolution, uav_id...)

obter os 14m (max_range do lidar) ou um pouco menos do caminho global
checar se há obstáculo nesse caminho local
se tiver:
    replaneja a rota apenas nesse caminho local, pra evitar o obstáculo
    desvia do obstáculo
    continua a rota global
'''
import numpy as np
from scipy.interpolate import CubicSpline
from uav.uav import UAV
from classic_path_planner.astar import AStar
from geometry.geometry import Geometry
from geometry.grid_map import GridMap

class LocalPathPlanner:
    def __init__(
            self,
            uav_id=1,
            grid_size=100,
            resolution=0.5,
            max_obstacle_dist=0.55,
        ) -> None:
        self.uav = UAV(uav_id)
        self.grid_size = grid_size
        self.resolution = resolution
        self.max_obstacle_dist = max_obstacle_dist
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

    def get_local_path(self, global_path, max_range=10.0):
        if len(global_path) < 2:
            return None
        
        uav_position = self.uav.uav_info.get_uav_position()
        
        # index do ponto mais próximo
        closest_point_idx = 0
        for i, point in enumerate(global_path):
            if Geometry.euclidean_distance(point, [uav_position.x, uav_position.y]) < 0.5:
                closest_point_idx = i
                break 
        
        local_path = []
        total_lenght = 0.0
        for i in range(closest_point_idx, len(global_path) - 1):
            current_point = global_path[i]
            next_point = global_path[i + 1]

            total_lenght += Geometry.euclidean_distance(current_point, next_point)
            local_path.append(current_point)

            if total_lenght > max_range:
                break

        return local_path
    
    def obstacle_found(self, global_path):
        obstacles = self.uav.map_environment.get_obstacles()
        x_obstacles  = np.array([p[0] for p in obstacles])
        y_obstacles  = np.array([p[1] for p in obstacles])
        
        local_path = np.array(self.get_local_path(global_path))
        x_local_path = np.array([p[0] for p in local_path])
        y_local_path = np.array([p[1] for p in local_path])

        for i in range(len(local_path)):
            distances = np.sqrt(
                (x_obstacles - x_local_path[i])**2 + (y_obstacles - y_local_path[i])**2)
            if np.any(distances < self.max_obstacle_dist):
                return True
        
        return False
    
    def avoid_obstacle(self, global_path):
        local_path = np.array(self.get_local_path(global_path))
        x_local_path = np.array([p[0] for p in local_path])
        y_local_path = np.array([p[1] for p in local_path])

        # array de obstáculos
        obstacles = self.uav.map_environment.get_obstacles()
        x_obstacles  = np.array([p[0] for p in obstacles])
        y_obstacles  = np.array([p[1] for p in obstacles])

        t = np.linspace(0, 1, len(x_local_path))
        spline_x = CubicSpline(t, x_local_path)
        spline_y = CubicSpline(t, y_local_path)

        local_x = spline_x(t)
        local_y = spline_y(t)

        for i in range(len(x_local_path)):
            # calcula a distância entre cada ponto da trajetória local e os obstáculos...
            distances = np.sqrt((x_obstacles - local_x[i])**2 + (y_obstacles - local_y[i])**2)

            # checa se há obstáculos muito próximos à trajetória local...
            if np.any(distances < self.max_obstacle_dist):
                # desvia
                local_x[i] += self.max_obstacle_dist
                local_y[i] += self.max_obstacle_dist

        # navega pela sub trajetoria...
        sub_trajectory = [(x, y) for x, y in zip(local_x, local_y)]
        self.uav.movements.goto_trajectory(sub_trajectory, wait=True)

        # volta a navegar pelo global path
        goal = local_path[-1]
        global_path = global_path.tolist()
        goal_idx = global_path.index(goal)
        self.uav.movements.goto_trajectory(global_path[goal_idx:])

    def run(self, global_path, max_range=10.0):
        obstacles = self.uav.map_environment.get_obstacles()

        local_path = self.get_local_path(global_path, max_range)
        start = local_path[0]
        goal = local_path[-1]

        # adiciona os obstáculos no grid...
        for obstacle in obstacles:
            x, y = round(obstacle[0] * 2) / 2, round(obstacle[1] * 2) / 2
            self.grid_map.set_value_from_xy_pos(x_pos=x, y_pos=y, val=1.0)
        
        # expande os obstáculos..
        self.grid_map.expand_grid()

        astar = AStar(self.grid_map)
        path = astar.find_path(
            start=start,
            goal=goal
        )

        # navega até o local path
        self.uav.movements.goto_trajectory(path, wait=True)

        # volta a navegar pelo global path
        goal_idx = global_path.index(goal)
        self.uav.movements.goto_trajectory(global_path[goal_idx:])
