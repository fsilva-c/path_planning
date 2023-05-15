import heapq
import math

class AStar:
    def __init__(self, grid_map):
        self.grid_map = grid_map

    def heuristic(self, a, b):
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def is_valid(self, node):
        x, y = node
        grid_ind = self.grid_map.calc_grid_index_from_xy_index(x, y)
        return 0 <= grid_ind < self.grid_map.ndata

    def is_obstacle(self, node):
        x, y = node
        return self.grid_map.check_occupied_from_xy_index(x, y)

    def neighbors(self, node):
        row, col = node
        candidates = [
            (row + 1, col), (row - 1, col), (row, col + 1), (row, col - 1),
            (row + 1, col + 1), (row + 1, col - 1), (row - 1, col + 1), (row - 1, col - 1)]
        results = filter(self.is_valid, candidates)
        return filter(lambda n: not self.is_obstacle(n), results)

    def find_path(self, start, goal):
        start = self.grid_map.get_xy_index_from_xy_pos(start[0], start[1])
        goal = self.grid_map.get_xy_index_from_xy_pos(goal[0], goal[1])

        open_list = []
        closed_set = set()
        came_from = {}
        g_scores = {start: 0}
        f_scores = {start: self.heuristic(start, goal)}

        heapq.heappush(open_list, (f_scores[start], start))

        while open_list:
            current = heapq.heappop(open_list)[1]
            # print(current)

            if current == goal:
                path = []
                while current in came_from:
                    x, y = current
                    path.append(self.grid_map.get_xy_pos_from_xy_index(x, y))
                    current = came_from[current]
                path.append(self.grid_map.get_xy_pos_from_xy_index(start[0], start[1]))
                path.reverse()
                return path

            closed_set.add(current)

            for neighbor in self.neighbors(current):
                tentative_g_score = g_scores[current] + 1

                if neighbor in closed_set and tentative_g_score >= g_scores.get(neighbor, float('inf')):
                    continue

                if tentative_g_score < g_scores.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_scores[neighbor] = tentative_g_score
                    f_scores[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    if neighbor not in closed_set:
                        heapq.heappush(open_list, (f_scores[neighbor], neighbor))

        return None
