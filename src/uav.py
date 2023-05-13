from movements import Movements
from uavinfo import UAVInfo
from hector import Hector

class UAV:
    def __init__(self, uav_id=1) -> None:
        self.movements = Movements(uav_id)
        self.uav_info = UAVInfo(uav_id)
        self.hector = Hector(uav_id)
        