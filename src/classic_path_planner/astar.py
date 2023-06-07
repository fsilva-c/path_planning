import heapq
from geometry.geometry import Geometry
from geometry.discrete_grid import DiscreteGrid
from scipy.spatial import KDTree

class Node:
    def __init__(self, position):
        self.position = position
        self.g = 0  # custo do caminho do nó inicial até este nó...
        self.h = 0  # heurística...
        self.f = 0  # f = g + h...
        self.parent = None

    def __lt__(self, other):
        return self.f < other.f

class AStar:
    def __init__(
            self,
            threshold: float, 
            dg: DiscreteGrid, 
            obstacles: list
        ) -> None:
        self.threshold = threshold
        self.dg = dg
        self.kd_tree = KDTree(obstacles)

    def heuristic(self, node, goal) -> float:
        # return Geometry.euclidean_distance(node.position, goal.position)
        return Geometry.manhattan_distance(node.position, goal.position)

    def is_valid(self, node: list) -> bool:
        distance, _ = self.kd_tree.query(self.dg.discrete_to_continuous(node), k=1)
        return distance >= self.threshold

    def get_neighbours(self, node: Node):
        x, y, z = node.position
        neighbours = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                for k in range(-1, 2):
                    if i == 0 and j == 0 and k == 0:
                        continue
                    new_node = Node((x + i, y + j, z + k))
                    if self.is_valid(new_node.position):
                        neighbours.append(new_node)
        return neighbours

    def find_path(self, start, goal):
        start_node = Node(self.dg.continuous_to_discrete(start))
        goal_node = Node(self.dg.continuous_to_discrete(goal))

        open_list = []
        closed_list = set()

        heapq.heappush(open_list, start_node)

        while open_list:
            current_node = heapq.heappop(open_list)
            closed_list.add(current_node)

            if current_node.position == goal_node.position:
                path = []
                while current_node:
                    path.append(self.dg.discrete_to_continuous(current_node.position))
                    current_node = current_node.parent
                path.reverse()
                return path

            neighbours = self.get_neighbours(current_node)
            for neighbour in neighbours:
                if neighbour in closed_list:
                    continue

                g = current_node.g + 1
                if neighbour in open_list:
                    if g < neighbour.g:
                        open_list.remove(neighbour)
                    else:
                        continue

                neighbour.g = g
                neighbour.h = self.heuristic(neighbour, goal_node)
                neighbour.f = g + neighbour.h
                neighbour.parent = current_node
                heapq.heappush(open_list, neighbour)

        return []
    