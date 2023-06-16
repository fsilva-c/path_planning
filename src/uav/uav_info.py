import rospy
from sensor_msgs.msg import Range, LaserScan, PointCloud, PointCloud2
from mavros_msgs.msg import State
from mrs_msgs.msg import PositionCommand, Float64Stamped, ControlManagerDiagnostics

class UAVInfo:
    def __init__(self, uav_id) -> None:
        self.uav_id = uav_id

        self.uav_pos = PositionCommand()
        self.diagnostics = ControlManagerDiagnostics()
        self.heading = Float64Stamped()
        self.state = State()
        self.garmin = Range()
        self.laser_scan = LaserScan()
        self.point_cloud = PointCloud()
        self.point_cloud_2 = PointCloud2()

        topic_prefix = f'/uav{self.uav_id}'
        rospy.Subscriber(f'{topic_prefix}/control_manager/position_cmd', PositionCommand, self.callback_position)
        rospy.Subscriber(f'{topic_prefix}/control_manager/heading', Float64Stamped, self.callback_heading)
        rospy.Subscriber(f'{topic_prefix}/garmin/range', Range, self.callback_garmin)
        rospy.Subscriber(f'{topic_prefix}/mavros/state', State, self.callback_state)
        rospy.Subscriber(f'{topic_prefix}/control_manager/diagnostics', ControlManagerDiagnostics, self.callback_diagnostics)
        rospy.Subscriber(f'{topic_prefix}/rplidar/scan', LaserScan, self.callback_laser_scan)
        rospy.Subscriber(f'{topic_prefix}/hector_mapping/slam_cloud', PointCloud, self.callback_point_cloud)
        rospy.Subscriber(f'{topic_prefix}/velodyne/scan', PointCloud2, self.callback_point_cloud_2)

    def callback_position(self, msg):
        self.uav_pos = msg

    def callback_heading(self, msg):
        self.heading = msg

    def callback_garmin(self, msg):
        self.garmin = msg
    
    def callback_diagnostics(self, msg):
        self.diagnostics = msg
    
    def callback_state(self, msg):
        self.state = msg

    def callback_laser_scan(self, msg):
        self.laser_scan = msg

    def callback_point_cloud(self, msg):
        self.point_cloud = msg

    def callback_point_cloud_2(self, msg):
        self.point_cloud_2 = msg

    def get_uav_position(self, tolist=False):
        return (self.uav_pos.position 
                if not tolist else
                [self.uav_pos.position.x, self.uav_pos.position.y, self.uav_pos.position.z])

    def get_heading(self):
        return self.heading.value
    
    def get_garmin_range(self):
        return self.garmin.range
    
    def get_active_tracker(self):
        return self.diagnostics.active_tracker
    
    def get_laser_scan(self):
        return self.laser_scan
    
    def get_point_cloud(self):
        return self.point_cloud

    def get_point_cloud_2(self):
        return self.point_cloud_2

    def is_armed(self):
        return self.state.armed

    def get_mode(self):
        return self.state.mode

    def get_motors_status(self):
        return self.diagnostics.motors