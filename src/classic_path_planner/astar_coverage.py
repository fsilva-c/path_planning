import math
from geometry.geometry import Geometry

class AStarCoverage:
    def __init__(self, grid_map):
        self.grid_map = grid_map
        self.unvisited_points = []  # Lista de pontos não visitados

    def heuristic(self, a, b):
        if len(self.unvisited_points) > 0:
            # Encontra o ponto não visitado mais próximo
            min_distance = math.inf
            for point in self.unvisited_points:
                distance = Geometry.euclidean_distance(a, point)
                if distance < min_distance:
                    min_distance = distance
            return min_distance * self.grid_map.resolution
        else:
            # Retornar a distância até a "casa" (ponto inicial)
            return Geometry.euclidean_distance(a, self.start) * self.grid_map.resolution

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
        directions = [
            (0, 1), (0, -1), (1, 0), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)]

        for dx, dy in directions:
            neighbor = (x + dx, y + dy)
            if self.is_valid(neighbor) and not self.is_obstacle(neighbor):
                neighbors.append(neighbor)
        return neighbors

    def find_path(self, start, unvisited_points):
        start = self.grid_map.get_xy_index_from_xy_pos(start[0], start[1])
        self.unvisited_points = [self.grid_map.get_xy_index_from_xy_pos(p[0], p[1]) for p in unvisited_points]
        self.start = self.grid_map.get_xy_index_from_xy_pos(start[0], start[1])   # Salvar o ponto inicial como a "casa"

        open_set = [start]  # Nodes to be evaluated
        came_from = {}  # Keeps track of the previous node in the optimal path
        g_score = {start: 0}  # Cost from start along the best known path
        f_score = {start: self.heuristic(start, start)}  # Estimated total cost from start to goal (start to "casa")

        while open_set:
            current = min(open_set, key=lambda node: f_score.get(node, math.inf))

            if current == self.start and not self.unvisited_points:  # Chegou à "casa" e visitou todos os pontos
                return self._reconstruct_path(came_from, current)

            open_set.remove(current)

            for neighbor in self.neighbors(current):
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)

                if tentative_g_score < g_score.get(neighbor, math.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, start)

                    if neighbor not in open_set:
                        open_set.append(neighbor)

            if current in self.unvisited_points:
                self.unvisited_points.remove(current)  # Remover ponto visitado

        return None  # No path found

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            x, y = self.grid_map.get_xy_pos_from_xy_index(current[0], current[1])
            path.append((x, y))
        path[0] = self.grid_map.get_xy_pos_from_xy_index(path[0][0], path[0][1])
        return path[::-1]
