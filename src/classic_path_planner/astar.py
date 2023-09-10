from itertools import product
from scipy.spatial import KDTree
from geometry.geometry import Geometry
from geometry.discrete_grid import DiscreteGrid

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
            obstacles: KDTree
        ) -> None:
        self.threshold = threshold
        self.dg = dg
        self.kd_tree = obstacles

    def heuristic(self, node, goal) -> float:
        return Geometry.euclidean_distance(node.position, goal.position)

    def is_valid(self, node: list) -> bool:
        continuous_node = self.dg.discrete_to_continuous(node)
        indices = self.kd_tree.query_ball_point(continuous_node, self.threshold * 2)
        return not indices

    def get_neighbours(self, node: Node):
        x, y, z = node.position
        deltas = [d for d in product((-1, 0, 1), repeat=3) if any(d)]
        for dx, dy, dz in deltas:
            new_node = Node((x + dx, y + dy, z + dz))
            if self.is_valid(new_node.position):
                yield new_node

    def find_path(self, start, goal):
        start_discrete = self.dg.continuous_to_discrete(start)
        goal_discrete = self.dg.continuous_to_discrete(goal)
        start_node = Node(start_discrete)
        goal_node = Node(goal_discrete)

        open_dict = {start_node.position: start_node}
        closed_set = set()

        while open_dict:
            current_node = min(open_dict.values(), key=lambda node: node.f)
            if current_node.position == goal_node.position:
                path = []
                while current_node:
                    path.append(self.dg.discrete_to_continuous(current_node.position))
                    current_node = current_node.parent
                return path[::-1]

            del open_dict[current_node.position]
            closed_set.add(current_node.position)

            for neighbour in self.get_neighbours(current_node):
                if neighbour.position in closed_set:
                    continue

                g = current_node.g + 1
                if neighbour.position in open_dict:
                    if g < open_dict[neighbour.position].g:
                        open_dict[neighbour.position].g = g
                        open_dict[neighbour.position].parent = current_node
                else:
                    neighbour.g = g
                    neighbour.h = self.heuristic(neighbour, goal_node)
                    neighbour.f = g + neighbour.h
                    neighbour.parent = current_node
                    open_dict[neighbour.position] = neighbour

        return []

    