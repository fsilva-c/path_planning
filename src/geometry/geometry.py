import math
import numpy as np

class Geometry:
    @staticmethod
    def euclidean_distance(p1, p2):
        x1, y1, z1 = p1
        x2, y2, z2 = p2
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    @staticmethod
    def norm(a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    @staticmethod
    def manhattan_distance(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])

    @staticmethod
    def laser_scan_to_coordinates(laser_scan, max_range=14.0):
        obstacles = []
        for i, range in enumerate(laser_scan.ranges):
            if range != math.inf and range <= max_range:
                angle = i * laser_scan.angle_increment
                x = range * math.cos(angle)
                y = range * math.sin(angle)
                obstacles.append([x, y])
        return obstacles
