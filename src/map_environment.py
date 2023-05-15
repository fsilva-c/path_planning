import math
import geometry
import numpy as np
from uav_info import UAVInfo

MAX_RANGE = 14.0

class MapEnvironment:
    def __init__(self, uav_id=1) -> None:
        self.uav_id = uav_id
        self.uav_info = UAVInfo(uav_id)
    
    def get_obstacles(self, max_range=MAX_RANGE):
        laser_scan = self.uav_info.get_laser_scan()
        obstacles = []
        for i, range in enumerate(laser_scan.ranges):
            if range != math.inf and range <= max_range:
                angle = laser_scan.angle_min + i * laser_scan.angle_increment
                x = range * math.cos(angle)
                y = range * math.sin(angle)
                obstacles.append([x, y])
        return obstacles

    def sort_by_drone_distance(self, l):
        uav_position = self.uav_info.get_uav_position()
        def distance(p):
            return geometry.euclidean_distance([uav_position.x, uav_position.y], p)
        l.sort(key=distance)

    def distance_to_closest_obstacle(self, max_range=MAX_RANGE):
        obstacles = self.get_obstacles(max_range=max_range)
        if obstacles:
            sorted_obstacles = sorted(obstacles, key=lambda p: (p[0] ** 2 + p[1] ** 2) ** 0.5)
            obstacle = sorted_obstacles[0]
            return geometry.euclidean_distance(obstacle, (0, 0))
        return None
    