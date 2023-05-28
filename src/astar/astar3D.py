import math
import heapq
from itertools import product
from geometry.geometry import Geometry
from geometry.discrete_grid import DiscreteGrid
from scipy.spatial import KDTree
import numpy as np

class AStar3D:
    def __init__(self, obstacles) -> None:
        self.dg = DiscreteGrid(1.0)
        self.obstacles = obstacles
        self.threshold = 1.4
        self.kd_tree = KDTree(self.obstacles)

    def heuristic(self, a, b):
        return Geometry.euclidean_distance(a, b)

    def is_valid(self, node):
        distances, _ = self.kd_tree.query(node, k=1)
        return np.all(distances >= self.threshold)
    
    def get_neighbors(self, node):
        x, y, z = node
        neighbors = []
        moves = [d for d in product((-1, 0, 1), repeat=3) if any(d)]
        
        for move in moves:
            new_x = x + move[0]
            new_y = y + move[1]
            new_z = z + move[2]

            new_node = (new_x, new_y, new_z)
            if self.is_valid(new_node):
                neighbors.append(new_node)

        return neighbors

    def find_path(self, start, goal):
        start = self.dg.continuous_to_discrete(start)
        goal = self.dg.continuous_to_discrete(goal)

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)

                if tentative_g_score < g_score.get(neighbor, math.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)

                    # Check if neighbor is already in the open set
                    neighbor_in_open_set = False
                    for _, node in open_set:
                        if node == neighbor:
                            neighbor_in_open_set = True
                            break

                    if not neighbor_in_open_set:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(self.dg.discrete_to_continuous(current))
        return path[::-1]