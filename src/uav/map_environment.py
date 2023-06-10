import tf
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry.geometry import Geometry
from geometry_msgs.msg import PointStamped
from geometry.laser_geometry import LaserProjection
from uav.uav_info import UAVInfo

class MapEnvironment:
    def __init__(self, uav_id: int=1) -> None:
        self.uav_id = uav_id
        self.uav_info = UAVInfo(uav_id)
    
    def get_obstacles_rplidar(self): # rplidar...
        laser_scan = self.uav_info.get_laser_scan()
        pc2_msg = LaserProjection().projectLaser(laser_scan)
        for p in pc2.read_points(pc2_msg, field_names=('x', 'y'), skip_nans=True):
            yield p

    def get_obstacles_realsense(self): # realsense...
        listener = tf.TransformListener()
        pc2_real_sense = self.uav_info.get_point_cloud_2()
        target_frame = 'uav1/fcu'
        
        listener.waitForTransform(
            target_frame, pc2_real_sense.header.frame_id, pc2_real_sense.header.stamp, rospy.Duration(1.0))

        for p in pc2.read_points(pc2_real_sense, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = p
            point = PointStamped()
            point.header.frame_id = pc2_real_sense.header.frame_id
            point.header.stamp = rospy.Time()
            point.point.x = x
            point.point.y = y
            point.point.z = z
            tf_point = listener.transformPoint(target_frame, point).point
            yield (tf_point.x, tf_point.y, tf_point.z)

    def distance_to_closest_obstacle(self):
       return min(self.uav_info.get_laser_scan().ranges)