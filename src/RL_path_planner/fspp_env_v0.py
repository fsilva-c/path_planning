import gymnasium as gym
from gymnasium import spaces

class FSPPEnv(gym.Env):
    def __init__(self) -> None:
        super().__init__()

        self.action_space = spaces.Discrete(6)
        self.reward_range = (float('-inf'), float('inf'))
    
    def step(self, action):
        match action:
            case 1:
                pass
    
    def reset(self):
        pass

    def render(self):
        ...
    
    def close(self):
        ...