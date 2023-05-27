import numpy as np

class GridMap3D:
    def __init__(self, resolution, dimension, center):
        self.resolution = resolution
        self.dimension = dimension
        self.center = center
        self.grid = np.zeros(dimension, dtype=int)

    def world_to_grid(self, position):
        indices = ((position - self.center) / self.resolution).astype(int)
        return indices
    
    def grid_to_world(self, indices):
        position = indices * self.resolution + self.center
        return position

    def is_valid_position(self, position):
        indices = self.world_to_grid(position)
        if np.any(indices < 0) or np.any(indices >= self.dimension):
            return False
        return self.grid[tuple(indices)] == 0

    def insert_obstacle(self, position):
        indices = self.world_to_grid(position)
        if np.all(indices >= 0) and np.all(indices < self.dimension):
            self.grid[tuple(indices)] = 1
            return True
        return False
