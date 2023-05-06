import rospy
from sensor_msgs.msg import Range
from mrs_msgs.msg import PositionCommand, Float64Stamped

class UAVInfo:
    def __init__(self, uav_id) -> None:
        self.uav_id = uav_id

        self.uav_pos = PositionCommand()
        self.heading = Float64Stamped()
        self.garmin = Range()

        topic_prefix = f'/uav{self.uav_id}'
        rospy.Subscriber(f'{topic_prefix}/control_manager/position_cmd', PositionCommand, self.callback_position)
        rospy.Subscriber(f'{topic_prefix}/control_manager/heading', Float64Stamped, self.callback_heading)
        rospy.Subscriber(f'{topic_prefix}/garmin/range', Range, self.callback_garmin)

    def callback_position(self, data):
        self.uav_pos = data

    def callback_heading(self, data):
        self.heading = data

    def callback_garmin(self, data):
        self.garmin = data

    def get_uav_position(self):
        return self.uav_pos.position

    def get_heading(self):
        return self.heading.value
    
    def get_garmin_range(self):
        return self.garmin.range
