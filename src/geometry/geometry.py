import math
import numpy as np
from scipy.interpolate import CubicSpline, BSpline

class Geometry:
    @staticmethod
    def euclidean_distance(a, b):
        distances = sum([(x2 - x1) ** 2 for x1, x2 in zip(a, b)])
        return math.sqrt(distances)

    @staticmethod
    def manhattan_distance(a, b):
        return sum(abs(x - y) for x, y in zip(a,b))

    @staticmethod
    def apply_cubic_spline(path, resolution=0.1):
        x, y, z = zip(*path)
        arange = np.arange(len(x))
        cs_x = CubicSpline(arange, x)
        cs_y = CubicSpline(arange, y)
        cs_z = CubicSpline(arange, z)
        interpolated_path = []
        t = 0.0
        while t < len(path) - 1:
            interpolated_point = (float(cs_x(t)), float(cs_y(t)), float(cs_z(t)))
            interpolated_path.append(interpolated_point)
            t += resolution
        return interpolated_path
    
    @staticmethod
    def remove_collinear_points(original_path: list) -> list:
        new_path = [original_path[0]]
        n = len(original_path)
        for i in range(1, n - 1):
            p1 = original_path[i - 1]
            p2 = original_path[i]
            p3 = original_path[i + 1]
            if not abs(
                (p2[1] - p1[1]) * (p3[0] - p2[0]) - (p2[0] - p1[0]) * (p3[1] - p2[1])
            ) < 0.001:
                new_path.append(p2)
        new_path.append(original_path[n - 1])
        return new_path
