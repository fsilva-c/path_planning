import heapq
from geometry.geometry import Geometry
from geometry.discrete_grid import DiscreteGrid

class PriorityQueue:
    def __init__(self):
        self.elements =[]

    def empty(self):
        return not self.elements

    def put(self, coordination, priority):
        heapq.heappush(self.elements, (priority, coordination))

    def get(self,):
        return heapq.heappop(self.elements)[1]

class AStar:
    def __init__(self, obstacles):
        self.obstacles = obstacles
        self.dg = DiscreteGrid(resolution=0.5)
        self.treshold = 0.9 / 0.5

    def is_obstacle(self, node):
        for p in self.obstacles:
            if Geometry.euclidean_distance(p, node) < self.treshold:
                return True
        return False

    def get_neighbors(self,current):
        x, y, z = current
        neighbors = set()

        movements = [
            (1, 0, 0), (0, -1, 0), (-1, 0, 0), 
            (0, 1, 0), (0, 0, 1), (0, 0, -1)]

        for dx, dy, dz in movements:
            new_x = x + dx
            new_y = y + dy
            new_z = z + dz

            if (not self.is_obstacle((new_x, new_y, new_y))):
                neighbors.add((new_x, new_y, new_z))

        return neighbors

    def heuristic(self, current, end_point):
        return Geometry.euclidean_distance(current, end_point)

    def find_path(self, start, goal):
        start = self.dg.continuous_to_discrete(start)
        goal = self.dg.continuous_to_discrete(goal)

        frontier = PriorityQueue()
        frontier.put(start, 0)

        came_from = dict()
        came_from[start] = None
        came_from[goal] = goal

        cost_value = dict()
        cost_value[start] = 0

        while not frontier.empty():
            current = frontier.get()
            if current == goal:
                break
            neighbors = self.get_neighbors(current)
            for next in neighbors:
                new_cost = self.heuristic(current, next)
                if next not in cost_value or new_cost < cost_value[next]:
                    cost_value[next] = new_cost 
                    priority = new_cost + self.heuristic(next, goal)
                    frontier.put(next,priority)
                    came_from[next] = current

        current = goal
        path = []
        while current != start:
            path.append(self.dg.discrete_to_continuous(current))
            current = came_from[current]
        path.append(self.dg.discrete_to_continuous(start))
        path.reverse()

        return path

'''

import math
from itertools import product
from geometry.geometry import Geometry

class AStar:
    def __init__(self, grid_map, obstacles) -> None:
        self.grid_map = grid_map
        self.obstacles = obstacles
        self.threshold = 1.4

    def heuristic(self, a, b):
        return Geometry.norm(a, b) * self.grid_map.resolution
        # return Geometry.manhattan_distance(a, b) * self.grid_map.resolution

    def is_valid(self, node):
        for p in self.obstacles:
            distance = Geometry.norm(p, self.grid_map.discrete_to_continuous(node))
            if distance < self.threshold:
                return False
        return True
    
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
        start = self.grid_map.continuous_to_discrete(start)
        goal = self.grid_map.continuous_to_discrete(goal)

        open_set = [start]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = min(open_set, key=lambda node: f_score.get(node, math.inf))

            if current == goal:
                return self._reconstruct_path(came_from, current)

            open_set.remove(current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)

                if tentative_g_score < g_score.get(neighbor, math.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)

                    if neighbor not in open_set:
                        open_set.append(neighbor)
        return None

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(self.grid_map.discrete_to_continuous(current))
        return path[::-1]
'''
