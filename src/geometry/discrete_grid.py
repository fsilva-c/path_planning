class DiscreteGrid:
    def __init__(self, resolution) -> None:
        self.resolution = resolution

    def continuous_to_discrete(self, pos):
        return (
            int((pos[0] + self.resolution * 0.5) / self.resolution),
            int((pos[1] + self.resolution * 0.5) / self.resolution),
            int((pos[2] + self.resolution * 0.5) / self.resolution)
        )

    def discrete_to_continuous(self, grid_pos):
        return (
            (grid_pos[0]) * self.resolution,
            (grid_pos[1]) * self.resolution,
            (grid_pos[2]) * self.resolution,
        )
