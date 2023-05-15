from movements import Movements
from uav_info import UAVInfo
from map_environment import MapEnvironment

class UAV:
    def __init__(self, uav_id=1) -> None:
        self.movements = Movements(uav_id)
        self.uav_info = UAVInfo(uav_id)
        self.map_environment = MapEnvironment(uav_id)
        