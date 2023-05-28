import numpy as np

class GridMap3D:
    def __init__(self, resolution, dimension, center):
        self.resolution = resolution
        self.dimension = dimension
        self.center = center
        self.grid = np.zeros(dimension)

    def world_to_grid(self, position):
        indices = [
            int((position[0] - self.center[0]) / self.resolution),
            int((position[1] - self.center[1]) / self.resolution),
            int((position[2] - self.center[2]) / self.resolution)
        ]
        return indices

    def grid_to_world(self, indices):
        position = [
            indices[0] * self.resolution + self.center[0],
            indices[1] * self.resolution + self.center[1],
            indices[2] * self.resolution + self.center[2]
        ]
        return position

    def is_valid_position(self, position):
        indices = self.world_to_grid(position)
        if (
            0 <= indices[0] < self.dimension[0] and
            0 <= indices[1] < self.dimension[1] and
            0 <= indices[2] < self.dimension[2]
        ):
            return True
        return False

    def is_valid_index(self, index):
        if (
            0 <= index[0] < self.dimension[0] and
            0 <= index[1] < self.dimension[1] and
            0 <= index[2] < self.dimension[2]
        ):
            return True
        return False

    def is_obstacle(self, index):
        # if not self.is_valid_index(index):
        #     return False
        x, y, z = index
        return self.grid[x, y, z] == 1

    def insert_obstacle(self, position):
        if not self.is_valid_position(position):
            return False
        x, y, z = self.world_to_grid(position)
        self.grid[x, y, z] = 1
        return True
