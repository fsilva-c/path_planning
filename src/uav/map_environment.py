import sensor_msgs.point_cloud2 as pc2
from geometry.laser_geometry import LaserProjection
from uav.uav_info import UAVInfo

class MapEnvironment:
    def __init__(self, uav_id: int=1) -> None:
        self.uav_id = uav_id
        self.uav_info = UAVInfo(uav_id)
    
    def get_obstacles(self):
        laser_scan = self.uav_info.get_laser_scan()
        pc2_msg = LaserProjection().projectLaser(laser_scan)
        for p in pc2.read_points(pc2_msg, field_names=('x', 'y'), skip_nans=True):
            x, y = p
            yield [x, y]
