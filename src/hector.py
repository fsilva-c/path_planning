import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud

class Hector:
    def __init__(self, uav_id=1) -> None:
        self.uav_id = uav_id

        self.occupancy_grid = OccupancyGrid()
        self.laser_scan = LaserScan()
        self.point_cloud = PointCloud()

        topic_prefix = f'/uav{self.uav_id}'
        rospy.Subscriber(f'{topic_prefix}/hector_mapping/map', OccupancyGrid, self.callback_occupancy_grid)
        rospy.Subscriber(f'{topic_prefix}/hector_mapping/slam_cloud', PointCloud, self.callback_point_cloud)
        rospy.Subscriber(f'{topic_prefix}/rplidar/scan', LaserScan, self.callback_laser_scan)

    def callback_occupancy_grid(self, msg):
        self.occupancy_grid = msg

    def callback_point_cloud(self, msg):
        self.point_cloud = msg
    
    def callback_laser_scan(self, msg):
        self.laser_scan = msg

    def get_occupancy_grid(self):
        return self.occupancy_grid

    def get_point_cloud(self):
        return self.point_cloud

    def get_laser_scan(self):
        return self.laser_scan
    