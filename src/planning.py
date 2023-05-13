import rospy
from octomap_msgs.msg import Octomap

class OctoMap:
    def __init__(self, uav_id=1) -> None:
        self.uav_id = uav_id
        self.octomap_full = Octomap()
        self.octomap_binary = Octomap()

        topic_prefix = f'/uav{self.uav_id}'
        rospy.Subscriber(f'{topic_prefix}/octomap_server/octomap_local_full', Octomap, self.callback_octomap_full)
        rospy.Subscriber(f'{topic_prefix}/octomap_server/octomap_local_binary', Octomap, self.callback_octomap_binary)

    def callback_octomap_full(self, data):
        self.octomap_full = data

    def callback_octomap_binary(self, data):
        self.octomap_binary = data

    def get_octomap_full(self):
        return self.octomap_full
    
    def get_octomap_binary(self):
        return self.octomap_binary
