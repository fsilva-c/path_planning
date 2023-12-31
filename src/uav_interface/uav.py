from uav_interface.movements import Movements
from uav_interface.uav_info import UAVInfo
from uav_interface.map_environment import MapEnvironment
from uav_interface.cam import Camera

class UAV:
    def __init__(self, uav_id: int=1) -> None:
        self.id = uav_id
        self.movements = Movements(uav_id)
        self.uav_info = UAVInfo(uav_id)
        self.map_environment = MapEnvironment(uav_id)
        self.camera = Camera(uav_id)
        