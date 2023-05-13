import numpy as np
from queue import PriorityQueue

class Astar:
    def __init__(self, grid, start, goal) -> None:
        self.grid = grid
        self.grid_size = grid.shape[0]
        self.start = start
        self.goal = goal

        self.move_costs = {'up': 1, 'down': 1, 'left': 1, 'right': 1}

    def heuristic(self, node):
        return np.sqrt((node[0] - self.goal[0]) ** 2 + (node[1] - self.goal[1]) ** 2)

    def get_neighbors(self, node):
        neighbors = []
        for move, cost in self.move_costs.items():
            if move == 'up' and node[0] > 0 and self.grid[node[0] - 1, node[1]] == 0:
                neighbors.append((node[0] - 1, node[1], cost))
            elif move == 'down' and node[0] < self.grid_size - 1 and self.grid[node[0] + 1, node[1]] == 0:
                neighbors.append((node[0] + 1, node[1], cost))
            elif move == 'left' and node[1] > 0 and self.grid[node[0], node[1] - 1] == 0:
                neighbors.append((node[0], node[1] - 1, cost))
            elif move == 'right' and node[1] < self.grid_size - 1 and self.grid[node[0], node[1] + 1] == 0:
                neighbors.append((node[0], node[1] + 1, cost))
        return neighbors

    def a_star(self):
        open_list = PriorityQueue()
        open_list.put((0, self.start))
        came_from = {}
        cost_so_far = {self.start: 0}

        while not open_list.empty():
            current = open_list.get()[1]

            if current == self.goal:
                break

            for neighbor in self.get_neighbors(current):
                new_cost = cost_so_far[current] + neighbor[2]
                if neighbor[0:2] not in cost_so_far or new_cost < cost_so_far[neighbor[0:2]]:
                    cost_so_far[neighbor[0:2]] = new_cost
                    priority = new_cost + self.heuristic(neighbor[0:2])
                    open_list.put((priority, neighbor[0:2]))
                    came_from[neighbor[0:2]] = current

        # monta o caminho a partir dos nós visitados...
        path = [self.goal]
        while path[-1] != self.start:
            path.append(came_from[path[-1]])

        return path[::-1]
    