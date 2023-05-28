from uav.movements import Movements
from uav.uav_info import UAVInfo
from uav.map_environment import MapEnvironment

class UAV:
    def __init__(self, uav_id: int=1) -> None:
        self.id = uav_id
        self.movements = Movements(uav_id)
        self.uav_info = UAVInfo(uav_id)
        self.map_environment = MapEnvironment(uav_id)
        