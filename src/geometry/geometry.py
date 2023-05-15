import math

class Geometry:
    @staticmethod
    def euclidean_distance(pointA, pointB):
        x1, y1 = pointA
        x2, y2 = pointB
        dx = x2 - x1
        dy = y2 - y1
        return math.sqrt(dx ** 2 + dy ** 2)

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
