import rospy
from sensor_msgs.msg import Range, LaserScan
from mrs_msgs.msg import PositionCommand, Float64Stamped, ControlManagerDiagnostics

class UAVInfo:
    def __init__(self, uav_id) -> None:
        self.uav_id = uav_id

        self.uav_pos = PositionCommand()
        self.diagnostics = ControlManagerDiagnostics()
        self.heading = Float64Stamped()
        self.garmin = Range()
        self.laser_scan = LaserScan()

        topic_prefix = f'/uav{self.uav_id}'
        rospy.Subscriber(f'{topic_prefix}/control_manager/position_cmd', PositionCommand, self.callback_position)
        rospy.Subscriber(f'{topic_prefix}/control_manager/heading', Float64Stamped, self.callback_heading)
        rospy.Subscriber(f'{topic_prefix}/garmin/range', Range, self.callback_garmin)
        rospy.Subscriber(f'{topic_prefix}/control_manager/diagnostics', ControlManagerDiagnostics, self.callback_diagnostics)
        rospy.Subscriber(f'{topic_prefix}/rplidar/scan', LaserScan, self.callback_laser_scan)

    def callback_position(self, data):
        self.uav_pos = data

    def callback_heading(self, data):
        self.heading = data

    def callback_garmin(self, data):
        self.garmin = data
    
    def callback_diagnostics(self, data):
        self.diagnostics = data
    
    def callback_laser_scan(self, msg):
        self.laser_scan = msg

    def get_uav_position(self):
        return self.uav_pos.position

    def get_heading(self):
        return self.heading.value
    
    def get_garmin_range(self):
        return self.garmin.range
    
    def get_active_tracker(self):
        return self.diagnostics.active_tracker
    
    def get_laser_scan(self):
        return self.laser_scan
