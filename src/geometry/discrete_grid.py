class DiscreteGrid:
    def __init__(self, resolution):
        self.resolution = resolution

    def discrete_to_continuous(self, discrete_coordinates):
        x, y, z = discrete_coordinates
        x_global = x * self.resolution
        y_global = y * self.resolution
        z_global = z * self.resolution
        return x_global, y_global, z_global

    def continuous_to_discrete(self, continuous_coordinates):
        x_global, y_global, z_global = continuous_coordinates
        x = int(x_global / self.resolution)
        y = int(y_global / self.resolution)
        z = int(z_global / self.resolution)
        return x, y, z
