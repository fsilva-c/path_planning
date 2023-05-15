import math
from geometry.geometry import Geometry

class AStar:
    def __init__(self, grid_map):
        self.grid_map = grid_map

    def heuristic(self, a, b):
        return (abs(a[0] - b[0]) + abs(a[1] - b[1])) * self.grid_map.resolution
        # return Geometry.euclidean_distance(a, b) * self.grid_map.resolution
        # return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) * self.grid_map.resolution

    def is_valid(self, node):
        x, y = node
        grid_ind = self.grid_map.calc_grid_index_from_xy_index(x, y)
        return 0 <= grid_ind < self.grid_map.ndata

    def is_obstacle(self, node):
        x, y = node
        return self.grid_map.check_occupied_from_xy_index(x, y)

    def neighbors(self, node):
        x, y = node
        neighbors = []
        # Define the possible movement directions: up, down, left, and right
        directions = [
            (0, 1), (0, -1), (1, 0), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dx, dy in directions:
            neighbor = (x + dx, y + dy)
            if self.is_valid(neighbor) and not self.is_obstacle(neighbor):
                neighbors.append(neighbor)
        return neighbors

    def find_path(self, start, goal):
        start = self.grid_map.get_xy_index_from_xy_pos(start[0], start[1])
        goal = self.grid_map.get_xy_index_from_xy_pos(goal[0], goal[1])

        open_set = [start]  # Nodes to be evaluated
        came_from = {}  # Keeps track of the previous node in the optimal path
        g_score = {start: 0}  # Cost from start along the best known path
        f_score = {start: self.heuristic(start, goal)}  # Estimated total cost from start to goal

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

        return None  # No path found

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            x, y = current
            path.append(self.grid_map.get_xy_pos_from_xy_index(x, y))
        path[0] = self.grid_map.get_xy_pos_from_xy_index(path[0][0], path[0][1])
        return path[::-1]