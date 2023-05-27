import math
import heapq
from time import perf_counter
from geometry.geometry import Geometry

class Node:
    def __init__(self, position):
        self.position = position
        self.g = 0  # custo do caminho do nó inicial até este nó...
        self.h = 0  # heurística...
        self.f = 0  # f = g + h...
        self.parent = None

    def __lt__(self, other):
        return self.f < other.f
    
class AStar3D:
    def __init__(self, obstacles=[]) -> None:
        self.obstacles = obstacles
        self.treshold = 1.0

    def heuristic(self, node, goal):
        dx = node.position[0] - goal.position[0]
        dy = node.position[1] - goal.position[1]
        dz = node.position[2] - goal.position[2]
        return math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
    
    def is_obstacle(self, node):
        for p in self.obstacles:
            if Geometry.euclidean_distance(p, node) < self.treshold:
                return True
        return False

    def is_valid(self, node):
        return not node in self.obstacles

    def get_neighbours(self, node):
        x, y, z = node.position
        neighbours = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                for k in range(-1, 2):
                    if i == 0 and j == 0 and k == 0:
                        continue
                    new_pos = (x + i, y + j, z + k)
                    if not self.is_obstacle(new_pos):
                        neighbours.append(Node(new_pos))
        return neighbours

    def find_path(self, start, goal):
        start_node = Node(start)
        goal_node = Node(goal)

        open_list = []
        closed_list = []

        heapq.heappush(open_list, start_node)

        while open_list:
            current_node = heapq.heappop(open_list)
            closed_list.append(current_node)

            if current_node.position == goal_node.position:
                path = []
                while current_node:
                    path.append(current_node.position)
                    current_node = current_node.parent
                path.reverse()
                return path

            neighbours = self.get_neighbours(current_node)
            for neighbour in neighbours:
                if neighbour in closed_list:
                    continue

                neighbour.g = current_node.g + 1
                neighbour.h = self.heuristic(neighbour, goal_node)
                neighbour.f = neighbour.g + neighbour.h
                neighbour.parent = current_node

                if neighbour in open_list:
                    if neighbour.g > current_node.g + 1:
                        neighbour.g = current_node.g + 1
                        neighbour.f = neighbour.g + neighbour.h
                        neighbour.parent = current_node
                else:
                    heapq.heappush(open_list, neighbour)

        return None
'''
obstacles = [(-9, -4, 5), (-12, 4, 0), (-9, 7, 5), (-12, -4, 5), (-7, -7, 0), (-11, 5, 5), (-7, -1, 5), (-7, -2, 5), (-10, 5, 2), (-10, 3, 5), (-7, 5, 1), (-12, -1, 5), (-12, -2, 5), (-8, 1, 0), (-11, -4, 5), (-10, -5, 0), (-12, 5, 1), (-8, 8, 5), (-9, 3, 3), (-8, -3, 5), (-10, -3, 0), (-9, -3, 5), (-9, 2, 1), (-8, -5, 5), (-12, 2, 5), (-7, 5, 3), (-7, 4, 5), (-9, 1, 5), (-9, -4, 0), (-8, 2, 1), (-10, 4, 5), (-8, 6, 1), (-11, 5, 0), (-10, 3, 0), (-9, 3, 5), (-9, -5, 1), (-11, -3, 5), (-11, 1, 5), (-7, 5, 5), (-7, -6, 5), (-11, -4, 0), (-10, 2, 1), (-9, -4, 2), (-12, 5, 5), (-8, -3, 0), (-11, 3, 5), (-11, 5, 2), (-7, 7, 5), (-11, 6, 1), (-8, 0, 5), (-9, -3, 0), (-7, 5, -1), (-13, 4, 0), (-9, 1, 0), (-7, 4, 0), (-8, -4, 1), (-8, 4, 5), (-10, 4, 0), (-9, -4, 4), (-8, 2, 5), (-8, 6, 5), (-9, 3, 0), (-11, 5, 4), (-10, 0, 5), (-11, 6, 3), (-7, 6, 2), (-9, -5, 5), (-10, 5, 4), (-7, 5, 0), (-10, -4, 1), (-8, -4, 3), (-7, -3, 5), (-10, 2, 5), (-14, 3, 1), (-7, 1, 5), (-10, 6, 5), (-9, 3, 2), (-9, 4, 1), (-11, 6, 5), (-7, 6, 4), (-12, 1, 5), (-8, 4, 0), (-7, 3, 5), (-8, -4, 5), (-9, 2, 3), (-12, 3, 5), (-11, 4, 1), (-9, 3, 4), (-8, -1, 5), (-8, -2, 5), (-14, 4, 1), (-8, 5, 1), (-10, -4, 5), (-10, 2, 0), (-9, 2, 5), (-9, -4, 1), (-9, 6, 5), (-7, -7, 5), (-11, 5, 1), (-10, -1, 5), (-10, -2, 5), (-9, 4, 5), (-10, 5, 1)]
start_point = (-19, 0, 2)
goal_point = (10, 5, 3)
start = perf_counter()
path = AStar3D(obstacles).find_path(start_point, goal_point)
print(f'Finalizou em: {perf_counter() - start}s')
print("Caminho encontrado:", path)
'''
