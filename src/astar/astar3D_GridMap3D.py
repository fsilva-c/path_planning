import heapq
from geometry.geometry import Geometry

class AStar3D:
    def __init__(self, grid_map):
        self.grid_map = grid_map

    def heuristic(self, current, goal):
        # return np.linalg.norm(np.array(current) - np.array(goal))
        return Geometry.euclidean_distance(current, goal)
        # return Geometry.manhattan_distance(current, goal)

    def get_neighbors(self, current):
        '''
        x, y, z = current
        neighbours = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                for k in range(-1, 2):
                    if i == 0 and j == 0 and k == 0:
                        continue
                    new_pos = (x + i, y + j, z + k)
                    if self.grid_map.is_valid_index(new_pos) and not self.grid_map.is_obstacle(new_pos):
                        neighbours.append(new_pos)
        return neighbours
        '''
        x, y, z = current
        neighbors = set()
        movements = [
            (1, 0, 0), (0, -1, 0), (-1, 0, 0), 
            (0, 1, 0), (0, 0, 1), (0, 0, -1)]
        for dx, dy, dz in movements:
            new_x = x + dx
            new_y = y + dy
            new_z = z + dz
            new_pos = (new_x, new_y, new_z)
            if self.grid_map.is_valid_index(new_pos) and not self.grid_map.is_obstacle(new_pos):
                neighbors.add(new_pos)
        return neighbors

    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        while current != start:
            path.append(self.grid_map.grid_to_world(current))
            current = came_from[current]
        path.append(self.grid_map.grid_to_world(start))
        path.reverse()
        return path

    def find_path(self, start, goal):
        start = tuple(self.grid_map.world_to_grid(start))
        goal = tuple(self.grid_map.world_to_grid(goal))

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, start, goal)

            neighbors = self.get_neighbors(current)
            for neighbor in neighbors:
                tentative_g_score = g_score[current] + 1 

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []
