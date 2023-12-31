import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Camera:
    def __init__(self, uav_id) -> None:
        self.uav_id = uav_id
        self.image  = Image()
        self.bridge   = CvBridge()

        rospy.Subscriber(f'/uav{self.uav_id}/mobius_front/image_raw', Image, self.callback_cam)

    def callback_cam(self, data):
        self.image = data

    def get_img_cam(self):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)
        return cv_image
    