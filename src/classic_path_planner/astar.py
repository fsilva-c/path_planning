import math
import numpy as np
from geometry.geometry import Geometry
from geometry.discrete_grid import DiscreteGrid2D
from scipy.spatial import KDTree

class AStar:
    def __init__(
            self, 
            threshold: float, 
            dg: DiscreteGrid2D, 
            obstacles: list
        ) -> None:
        self.threshold: list = threshold
        self.dg = dg
        self.obstacles = obstacles
        self.kd_tree = KDTree(self.obstacles)

    def heuristic(self, a: list, b: list) -> float:
        return Geometry.manhattan_distance(a, b)
        # return Geometry.euclidean_distance(a, b)

    def is_valid(self, node: list) -> bool:
        distances, _ = self.kd_tree.query(self.dg.discrete_to_continuous(node), k=1)
        return np.all(distances >= self.threshold)

    def neighbors(self, node: list) -> list:
        x, y = node
        neighbors = []
        directions = [
            (0, 1), (0, -1), (1, 0), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        for dx, dy in directions:
            neighbor = (x + dx, y + dy)
            if self.is_valid(neighbor):
                neighbors.append(neighbor)
        return neighbors

    def find_path(self, start: list, goal: list) -> list:
        start = self.dg.continuous_to_discrete(start[:2])
        goal = self.dg.continuous_to_discrete(goal[:2])

        open_set = [start]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = min(open_set, key=lambda node: f_score.get(node, math.inf))

            if current == goal:
                return self._reconstruct_path(came_from, current)

            open_set.remove(current)

            for neighbor in self.neighbors(current):
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)

                if tentative_g_score < g_score.get(neighbor, math.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)

                    if neighbor not in open_set:
                        open_set.append(neighbor)

        return []

    def _reconstruct_path(self, came_from: dict, current: list) -> list:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(self.dg.discrete_to_continuous(current))
        path[0] = self.dg.discrete_to_continuous(path[0])
        return path[::-1]
    