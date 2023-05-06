from movements import Movements

class UAV:
    def __init__(self, uav_id=1) -> None:
        self.movements = Movements(uav_id)
        