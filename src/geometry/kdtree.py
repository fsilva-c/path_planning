import math

class Node:
    def __init__(self, sphere, left=None, right=None):
        self.sphere = sphere
        self.left = left
        self.right = right

class KDTree:
    def __init__(self, spheres):
        self.root = self._build_kd_tree(spheres, depth=0)

    def _build_kd_tree(self, spheres, depth):
        if not spheres:
            return None

        axis = depth % 3  # Alternando os eixos x, y e z
        spheres.sort(key=lambda sphere: sphere.center[axis])
        median = len(spheres) // 2

        left_subtree = self._build_kd_tree(spheres[:median], depth + 1)
        right_subtree = self._build_kd_tree(spheres[median + 1:], depth + 1)

        return Node(spheres[median], left_subtree, right_subtree)

    def is_point_near_sphere(self, point, threshold):
        return self._is_point_near_sphere(self.root, point, threshold, depth=0)

    def _is_point_near_sphere(self, node, point, threshold, depth):
        if node is None:
            return False

        axis = depth % 3
        current_sphere = node.sphere

        if abs(current_sphere.center[axis] - point[axis]) > threshold:
            # O ponto está longe o suficiente no eixo atual, podemos pular para o próximo nó.
            return self._is_point_near_sphere(node.left if point[axis] < current_sphere.center[axis] else node.right, point, threshold, depth + 1)
        
        # Caso contrário, calculamos a distância entre o ponto e a esfera.
        distance = math.sqrt(sum((current_sphere.center[i] - point[i]) ** 2 for i in range(3)))

        return distance <= (current_sphere.radius * 2 - threshold)

class Sphere:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
