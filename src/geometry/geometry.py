import math
import numpy as np

class Geometry:
    @staticmethod
    def euclidean_distance(a, b):
        distances = sum([(x2 - x1) ** 2 for x1, x2 in zip(a, b)])
        return math.sqrt(distances)

    @staticmethod
    def manhattan_distance(a, b):
        return sum(abs(x - y) for x, y in zip(a,b))
    
    @staticmethod
    def norm(a, b):
        return np.linalg.norm(np.array(a) - np.array(b))
