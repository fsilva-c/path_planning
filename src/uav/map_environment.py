import math
import struct
import numpy as np
from geometry.laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2
from geometry.geometry import Geometry
from uav.uav_info import UAVInfo

MAX_RANGE = 14.0

class MapEnvironment:
    def __init__(self, uav_id=1) -> None:
        self.uav_id = uav_id
        self.uav_info = UAVInfo(uav_id)

    def get_obstacles2(self):
        cos_sin_map = np.array([[]])
        laser_scan = self.uav_info.get_laser_scan()
        
        N = len(laser_scan.ranges)
        ranges = np.array(laser_scan.ranges)
        angles = laser_scan.angle_min + np.arange(N) * laser_scan.angle_increment
        cos_sin_map = np.array([np.cos(angles), np.sin(angles)])

        obstacles = ranges * cos_sin_map
        # obstacles = obstacles[np.isfinite(obstacles)]
        points = []
        for i in range(N):
            point = obstacles[:, i].tolist()
            x, _ = point
            if math.isfinite(x):
                points.append(point)

        return points
    
    def get_obstacles(self, max_range=MAX_RANGE):
        laser_scan = self.uav_info.get_laser_scan()
        laser_projection = LaserProjection()
        pc2_msg = laser_projection.projectLaser(laser_scan)
        for p in pc2.read_points(pc2_msg, field_names=('x', 'y'), skip_nans=True):
            x, y = p
            yield [x, y]

        '''
        # obstacles = []
        for i, range in enumerate(laser_scan.ranges):
            if range != math.inf and range <= max_range:
                angle = laser_scan.angle_min + i * laser_scan.angle_increment
                x = range * math.cos(angle)
                y = range * math.sin(angle)
                yield [x, y]
                # obstacles.append([x, y])
        # return obstacles
        '''
    
    def unpack_point_cloud_2(self):
        cloud = self.uav_info.get_point_cloud_2()
        data = cloud.data
        point_step = cloud.point_step

        x_offset = cloud.fields[0].offset
        y_offset = cloud.fields[1].offset
        z_offset = cloud.fields[2].offset

        points = []
        for i in range(0, len(data), point_step):
            x = struct.unpack_from('f', data, i + x_offset)[0]
            y = struct.unpack_from('f', data, i + y_offset)[0]
            z = struct.unpack_from('f', data, i + z_offset)[0]
            points.append((x, y, z))

        return points

    def sort_by_drone_distance(self, l):
        uav_position = self.uav_info.get_uav_position()
        def distance(p):
            return Geometry.euclidean_distance([uav_position.x, uav_position.y], p)
        l.sort(key=distance)

    def distance_to_closest_obstacle(self, max_range=MAX_RANGE):
        obstacles = self.get_obstacles(max_range=max_range)
        if obstacles:
            sorted_obstacles = sorted(obstacles, key=lambda p: (p[0] ** 2 + p[1] ** 2) ** 0.5)
            obstacle = sorted_obstacles[0]
            return Geometry.euclidean_distance(obstacle, (0, 0))
        return None
    